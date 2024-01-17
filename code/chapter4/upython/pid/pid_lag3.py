# pid_lag3.py
# dew.ninja  June 2023
# implement PID controller with
# LAG3 plant simulation on timer
#

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

pwm_out = PWM(Pin(16))  # output to lag3 board
pwm_out.freq(5000) # set PWM frequency to 5 KHz
dac_out = DAC(Pin(25))

y_in = ADC(Pin(39)) # ADC1_CH3
y_in.atten(ADC.ATTN_11DB) # range 0 - 3.6 V

adc2v = 3.6/ADCMAX  # max of ADC is set to 3.6 v by the package
v2pwm = int(PWMMAX/3.3)
v2dac = int(DACMAX/3.3)


led = Pin(2, Pin.OUT)
rled = PWM(Pin(19))  # red led
gled = PWM(Pin(18))  # green led
bled = PWM(Pin(17))  # blue led


T = 0.05  # sampling period
T_ms = int(1000*T)
t_data = 0  # time data output

# values for these PID coefficients are computed in PID_update
bi = 0
ad = 0
bd = 0
bt = 0

# controller states
ep = 0  # error for proportional term
e1 = 0  # true error (for integral term)
e0 = 0
eus1 = 0 # error for back calculation term 
eus0 = 0
ed1 = 0  # error for derivative term
ed0 = 0
ui1 = 0  # integral-term outputs
ui0 = 0
ud1 = 0  # derivative-term outputs
ud0 = 0

# PID parameters
kp = 4.8  # PID gains
ki = 2.74
kd = 2.1
kt = 0  # back calculaiton gain
wp = 1  # proportional weight
wd = 1  # derivative weight
N = 50  # derivative filter coefficient


r = 0.5  # ref. cmd
u = r   # plant input
ulim = r
u_pwm = u*v2pwm
y = 0.0  # plant output
plantsim = 0 # plant simulation mode
datasize = 200  # number of points in output capture
capture = False  # capture mode 0 = off, 1 = on
capture_flag = False # indicates start/stop of data transfer
data_idx = 0  # data index
# ---- controller -----
feedback = 1  # 0 = open-loop, 1 = close-loop

# These functions are not included in book
# adjust RGB color according to plant output y
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

# compute PID coefficients, also update freeboard
def PID_update():
    global ad, bd, bi, bt
    bi = 0.5*T*ki
    bt = 0.5*T*kt
    ad1 = 1+0.5*N*T
    ad2 = 0.5*N*T - 1
    ad = -ad2/ad1
    bd = kd*N/ad1
    #update_dashboard()  


# PID controller function
def PID_controller(r,y):
    global e1,e0,ed1,ed0, eus1,eus0, ui1, ui0, ud1, ud0, u
    
    # state transfer
    e1 = e0
    ed1 = ed0
    eus1 = eus0
    
    ui1 = ui0
    ud1 = ud0
    # compute errors for each term
    e0 = r - y
    ep0 = wp*r - y # weighted proportional error
    ed0 = wd*r - y # weighted derivative error
    
    up0 = kp*ep0 # output of P term
    ui0 = ui1 +bi*(e0+e1) + bt*(eus0+eus1) # output of I term
    ud0 = ad*ud1 +bd*(ed0 - ed1) # output of D term
    u = up0 + ui0 + ud0
    u_lim = u
    if u > UMID:
        eus0 = UMID - u  # compute error for back calculation term
        u_lim = UMID         # limit u to UMID
    elif u < -UMID:
        eus0 = -u - UMID  # compute error for back calculation term
        u_lim = -UMID         # limit u to -UMID
    u_lim += UMID
    return u_lim
    
    

def timer_isr(event):
    global a,b,T,u,u_states, y_states, capture_flag,t_data, \
           data_idx,datasize, prompt, y
    led.value(not led.value())
    #y_adc = y_in.read()  # read from ADC
    #y1 = y_adc*adc2v  # convert to volt
    if plantsim:
        y = lag3(a,b,T,u, u_states, y_states) # simulation. in volt
    else:
        y = y_in.read_uv()*1e-6 # from electronics
    y2rgb() # just lid RGB LED. Not discussed in book.
    if feedback:
        ulim = PID_controller(r,y)
    else:
    # open-loop 
        u = r
        ulim = u
    u_pwm = int(ulim*v2pwm) # v2pwm = int(PWMMAX/3.3)
    pwm_out.duty(u_pwm)  # send output via PWM pin 16
    u_dac = int(ulim*v2dac) # v2dac = int(DACMAX/3.3)
    dac_out.write(u_dac) # also send output to DAC1
    
    if capture_flag:
        print("[{},{},{},{}],".format(round(t_data,2),r,y,ulim))        
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
    global r,plantsim,datasize,capture,capture_flag,feedback,T,T_ms,a,b, prompt, \
           kp,ki,kd,kt,wp,wd,N
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
            PID_update()
            timer.init(period=T_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)            
    elif cmdstr.lower() == "kp":
        if noparm==1:
            print("Current kp = {}".format(kp))
        else:            
            kp = float(parmstr)
            if kp > 100: # set maximum kp
                kp = 100
            elif kp < 0: # minimum kp
                kp = 0
            PID_update()
            # update_dashboard()

    elif cmdstr.lower() == "ki":
        if noparm==1:
            print("Current ki = {}".format(ki))
        else:            
            ki = float(parmstr)
            if ki > 100: # set maximum ki
                ki = 100
            elif ki < 0: # minimum ki
                ki = 0
            PID_update()
            #update_dashboard()

    elif cmdstr.lower() == "kd":
        if noparm==1:
            print("Current kd = {}".format(kd))
        else:            
            kd = float(parmstr)
            if kd > 100: # set maximum kd
                kd = 100
            elif kd < 0: # minimum kd
                kd = 0
            PID_update()
            #update_dashboard()

    elif cmdstr.lower() == "kt":
        if noparm==1:
            print("Current kt = {}".format(kt))
        else:            
            kt = float(parmstr)
            if kt > 100: # set maximum kt
                kt = 100
            elif kt < 0: # minimum kt
                kt = 0
            PID_update()
            #update_dashboard()

    elif cmdstr.lower() == "wp":
        if noparm==1:
            print("Current wp = {}".format(wp))
        else:            
            wp = float(parmstr)
            if wp > 10: # set maximum wp
                wp = 10
            elif wp < 0: # minimum wp
                wp = 0
            PID_update()
            #update_dashboard()

    elif cmdstr.lower() == "wd":
        if noparm==1:
            print("Current wd = {}".format(wd))
        else:            
            wd = float(parmstr)
            if wd > 10: # set maximum wd
                wd = 10
            elif wd < 0: # minimum wd
                wd = 0
            PID_update()
            #update_dashboard()

    elif cmdstr.lower() == "n":
        if noparm==1:
            print("Current N = {}".format(N))
        else:            
            N = float(parmstr)
            if N > 200: # set maximum N
                N = 200
            elif N < 2: # minimum N
                N = 2
            PID_update()
            #update_dashboard()
    elif cmdstr.lower() == "y":
        print(y)            
    else:
        print("Invalid command")    


def user_input():
    if prompt:
        sys.stdout.write('\nEnter command : ')
    user_str = sys.stdin.readline()
    newline_char = sys.stdin.readline()
    cmdInt(user_str)

timer.init(period=T_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)
PID_update()

while True:
    user_input()

