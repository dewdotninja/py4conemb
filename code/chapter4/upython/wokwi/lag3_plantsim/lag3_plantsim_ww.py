# lag3_plantsim_ww.py
# dew.ninja  June 2023
# implement 
# LAG3 plant simulation on timer
# for Wokwi

#import machine
#import time
from utime import sleep_ms 
from machine import Pin,Timer, PWM, ADC, DAC
import sys

prompt = True  # set prompt off/on so that it won't get in the way.

# range variables
PWMMAX = 1023  # maximum PWM value
PWMMID = 511
PWMMIN = 0
DACMAX = 255
DACMID = 127
DACMIN = 0
ADCMAX = 4095
ADCMID = 2047
ADCMIN = 0
UMAX = 3.2  # limit max controller output to 3.2 vols
UMID = 1.65
UMIN = 0.1 # min controller output

pwm_out = PWM(Pin(5))  # output to lag3 board. Change from 16 to 5 for Wokwi
pwm_out.freq(1000) # set PWM frequency to 1 KHz (lower from 5KHz)
dac_out = DAC(Pin(25)) # this doesn't work on Wokwi

y_in = ADC(Pin(39)) # ADC1_CH3. Show as VN on Wokwi
y_in.atten(ADC.ATTN_11DB) # range 0 - 3.6 V

adc2v = 3.6/ADCMAX  # max of ADC is set to 3.6 v by the package
v2pwm = int(PWMMAX/3.3)
v2dac = int(DACMAX/3.3)


led = Pin(2, Pin.OUT)
rled = PWM(Pin(19))  # red led
gled = PWM(Pin(18))  # green led
bled = PWM(Pin(21))  # blue led. Originally 17

# ---- RGB LED function ------
def y2rgb():
    global y
    if y <= 1.5:
        yt = y/1.5
        rval = 0
        bval = int(PWMMAX*(1-yt))
        gval = int(PWMMAX*yt)
    else:
        yt = (y-1.5)/1.5
        if yt>1:
            yt = 1
        bval = 0
        gval = int(PWMMAX*(1-yt))
        rval = int(PWMMAX*yt)
    lidRGBled(rval, gval, bval)

def lidRGBled(rval, gval, bval):
    if rval>1023:
        rval = 1023
    elif rval<0:
        rval = 0
    if gval>1023:
        gval = 1023
    elif gval<0:
        gval = 0
    if bval>1023:
        bval = 1023
    elif bval<0:
        bval = 0
    
    rled.duty(rval)
    gled.duty(gval)
    bled.duty(bval)
# ----------------------------------

T = 0.05  # sampling period
T_ms = int(1000*T)
t_data = 0  # time data output

r = 0.5  # ref. cmd
u = r   # plant input
y = 0.0  # plant output
plantsim = 0 # plant simulation mode
datasize = 200  # number of points in output capture
capture = False  # capture mode 0 = off, 1 = on
capture_flag = False # indicates start/stop of data transfer
data_idx = 0  # data index
# ---- controller -----
feedback = 1  # 0 = open-loop, 1 = close-loop

# function for computing LAG3 coefficients
def lag3_init(T):
    a = 2+T
    b = T-2
    return a,b

a,b = lag3_init(T)
# variables for plant simulation 

y_states = [0.0]*6
u_states = [0.0]*6

timer=Timer(1)

# plant simulation
def lag3(a,b,T, u, u_states, y_states):
    for k in range(3):
        y_states[2*k] = y_states[2*k+1]
        u_states[2*k] = u_states[2*k+1]
        if k == 0:
            u_states[2*k+1] = u
        else:
            u_states[2*k+1] = y_states[2*k-1]
        y_states[2*k+1] = (1/a)*(-b*y_states[2*k]+T*(u_states[2*k+1]+u_states[2*k]))
    return y_states[5]

def timer_isr(event):
    global a,b,T,u,u_states, y_states, capture_flag,t_data, \
           data_idx,datasize, prompt
    led.value(not led.value())
    #y_adc = y_in.read()  # read from ADC
    #y1 = y_adc*adc2v  # convert to volt
    #y1 = y_in.read_uv()*1e-6
    y1 = y_in.read()*adc2v  # works better on Wokwi 
    y2 = lag3(a,b,T,u, u_states, y_states) # smulation. in volt
    # open-loop test
    u = r
    ulim = u
    u_pwm = int(ulim*v2pwm) # v2pwm = int(PWMMAX/3.3)
    pwm_out.duty(u_pwm)  # send output via PWM pin 16
    #u_dac = int(ulim*v2dac) # v2dac = int(DACMAX/3.3)
    #dac_out.write(u_dac) # also send output to DAC1
    
    if capture_flag:
        print("[{},{},{},{}],".format(round(t_data,2),r,y1,y2))        
        #print("[{},{},{},{},{}],".format(round(t_data,2),r,y,u,ulim))
        t_data+=T
        data_idx+=1
        if data_idx==datasize:
            data_idx = 0
            t_data = 0
            capture_flag = False  # stop capture data 
            print("])")  # end of np.array([ command
            prompt=True  # prompt on

# helper function for cmdInt()
# separate command and parameter
# at "="
def splitCmd(userstr):
    result = userstr.find("=")
    if result == -1:
        noparm = 1
        cmdstr = userstr.strip()
        parmstr = '' # not used
    else:
        noparm = 0
        splitstr = userstr.split("=")
        cmdstr = splitstr[0].strip()
        parmstr = splitstr[1].strip()
    return cmdstr, parmstr, noparm

#command interpreter function
def cmdInt(userstr):
    global r,plantsim,datasize,capture,capture_flag,feedback,T,T_ms,a,b, prompt
    cmdstr, parmstr, noparm = splitCmd(userstr)

    if cmdstr.lower() == "r" or cmdstr.lower() == "step":
        # toggle_mode = 0  # turn off toggle mode
        
        if noparm==1: # retrieve current r 
            print(r)
        else:
            r_tmp = float(parmstr)
            if r_tmp > 3.0:  # limit range 
                r_tmp = 3.0
            elif r_tmp < 0.0: 
                r_tmp = 0.0
            r = r_tmp
        if capture:  # capture mode on
            capture_flag = True
            print("datamat = np.array([")  # head of data matrix
            prompt=False # turn prompt off 
        # update_dashboard()
    elif cmdstr.lower() == "psim":
        if noparm==1:
            print(plantsim)
        else:            
            plantsim = int(parmstr)
            if plantsim > 1:
                plantsim = 1
            elif plantsim < 0:
                plantsim = 0
    elif cmdstr.lower() == "capture":
        if noparm==1:
            print(capture)
        else:            
            capture = int(parmstr)
            if capture > 1:
                capture = 1
            elif capture < 0:
                capture = 0
            
    elif cmdstr.lower() == "datasize":
        if noparm==1:
            print(datasize)
        else:            
            datasize = int(parmstr)
            if datasize > 2000: # set maximum datasize
                datasize = 2000
            elif datasize < 10: # minimum datasize
                datasize = 10
    elif cmdstr.lower() == "feedback":
        if noparm==1:
            print(feedback)
        else:            
            feedback = int(parmstr)
            if feedback > 1:
                feedback = 1
            elif feedback < 0:
                feedback = 0
    elif cmdstr.lower() == "t":
        if noparm==1:
            print("Current T = {}".format(T))
        else:            
            T = float(parmstr)
            if T > 1: # set maximum T
                T = 1
            elif T < 0.01: # minimum T
                T = 0.01
            T_ms = int(1000*T)
            a,b = lag3_init(T)
            timer.init(period=T_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)
            
    else:
        print("Invalid command")    


def user_input():
    if prompt:
        #sys.stdout.write('\nEnter command : ')
		print('\nEnter command : ')
    #user_str = sys.stdin.readline()
    #newline_char = sys.stdin.readline()
    user_str = input()
    cmdInt(user_str)

timer.init(period=T_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)

while True:
    user_input()


