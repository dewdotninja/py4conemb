# iot_controller4.py
# dew.ninja  June 2023
# modified from iot_controller3.py
# - add nodered support
# - add function to save parameters to shadow, and load
# from shadow.
# - add function to send message to dashboard directly
# - make it an IoT controller
# - add state-feedback with integrator
# - implement output feedback controller with
# LAG3 plant simulation on timer
# keep PID control as option

#import machine
import time
from utime import sleep_ms 
from machine import Pin,Timer, PWM, ADC, DAC
import sys

# --- added for iot development-----
from umqtt.robust import MQTTClient
import ujson
import network

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = ""
MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""
PUBLISH_PERIOD = 2000  # milliseconds
shadow_data = {'r': 0, 'y': 0, 'u': 0}
#time_current = 0  # variables to control publishing period
time_prev = 0 # variables to control publishing period
private_msg = "" # added in iot_controller3.py

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifi_ssid, wifi_pwd)
        while not wlan.isconnected():
            pass
    #print('network config:', wlan.ifconfig())

def init_client():
    global client

    print("Trying to connect to mqtt broker.")

    try:
        client = MQTTClient(MQTT_CLIENT, MQTT_BROKER, port=1883, user=MQTT_USER,
                            password=MQTT_PWD)
        client.connect()
        print("Connected to ",MQTT_BROKER)
        topic_sub = b"@msg/cmd"
        print("Subscribed to ",topic_sub)
        #added in iot_controller3.py 
        private_topic_sub = b"@private/#"
        print("Subscribed to ",private_topic_sub)
        
        client.set_callback(sub_cb)
        client.subscribe(topic_sub)
        client.subscribe(private_topic_sub)
        
    except:
        print("Trouble to init mqtt.") 


def sub_cb(topic, msg):
    global private_msg, cmdtime_prev
    print((topic, msg))
    if topic == b'@msg/cmd':
        rcvdstrs = str(msg).split("'") # get rid of b'
        rcvdstr = rcvdstrs[1]
        cmdInt(rcvdstr)
#         # this delay is needed for nodered implementation
#         cmdtime_current = time.ticks_ms()
#         delta_cmdtime = cmdtime_current - cmdtime_prev
#         if delta_cmdtime > CMD_DELAY:
#             cmdtime_prev = cmdtime_current
#             cmdInt(rcvdstr)
    # added in iot_controller3.py to retrieve shadow data 
    elif topic == b'@private/shadow/data/get/response':
        print(msg)
        private_msg = str(msg)

online = True
if online:
    wifi_connect()  # connect to WiFi network
    init_client()

button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch
button_state = True

def update_dashboard():    
    updatestr = ("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}"
                 .format(T, plantsim, datasize, capture, feedback,
                controller,autotune,lsid,kp,ki,kd,kt,wp,wd,N))
    print(updatestr)
    client.publish('@msg/update', updatestr)    


# --------------------------------

# ------- added in iot_controller3.py -------------
# --- this segment is for shadow save & load feature ----
initparm = 1  # 0 = init parameters from this script
              # 1 = init parameters from shadow
initparm_data = {'initparm' : initparm}
parms_update = 0  # global variable to update parameters

# parameter dictionary to write to shadow
parms_data = {'controller': 0, 'plantsim': 0 ,'datasize': 0,'capture': 0,
    'feedback': 0,'kp':0, 'ki':0,'kd':0,'kt':0,'wp':0,'wd':0,'N':0, 'T':0}
private_request_flag = False
cnt = 0  # counter used to resetting private_request_flag

def save_parms_to_shadow():
    global parms_data
    # write parameter values to parm_data
    # parms_data = {'controller':controller,'plantsim': plantsim ,
    # 'datasize': datasize,
    # 'capture': capture, 'feedback': feedback,'kp':kp,
    #'ki':ki,'kd':kd,'kt':kt,'wp':wp,'wd':wd,'N':N, 'T':T}
    parms_data['controller'] = controller
    parms_data['plantsim'] = plantsim
    parms_data['datasize'] = datasize
    parms_data['capture'] = capture
    parms_data['feedback'] = feedback
    parms_data['kp'] = kp
    parms_data['ki'] = ki
    parms_data['kd'] = kd
    parms_data['kt'] = kt
    parms_data['wp'] = wp
    parms_data['wd'] = wd
    parms_data['N'] = N
    parms_data['T'] = T
   
    # write to shadow
    print("Parameters saved to shadow")
    payload = ujson.dumps({"data": parms_data})
    client.publish("@shadow/data/update", payload)

def load_parms_from_shadow():
    global private_request_flag
    # read parameters from shadow and update
    # publish empty string to @shadow/data/get
    client.publish("@shadow/data/get", " ")
    sleep_ms(2000) # needs some delay time before data available
    private_request_flag = True
    
def get_private_message():
    return private_msg

# --------------------------        

def split_parms(update=0):
    global parms_update,private_request_flag, cnt,initparm,T,T_ms,plantsim,datasize,capture,feedback,kp,ki,kd,kt,wp,wd,N
    sleep_ms(2000) # more delay
    #msg = netpie.get_private_message()
    msg = get_private_message()
    print("Read attempt #"+str(cnt+1))
    print(msg)

    cnt+=1
    if (cnt == 1): # have to call get_private_message() 2 times for it to work!
        private_request_flag = False
        cnt = 0
        print("Parameters from NETPIE shadow")
        # extract initparm
        selected_data = msg.split("initparm\":")
        value = selected_data[1].split(',')[0]
        print("initparm = "+value)
        initparm = int(value)                

        # extract plant simulation
        selected_data = msg.split("controller\":")
        value = selected_data[1].split(',')[0]
        print("_controller = "+value)
        _controller = int(value)


        # extract plant simulation
        selected_data = msg.split("plantsim\":")
        value = selected_data[1].split(',')[0]
        print("_plantsim = "+value)
        _plantsim = int(value)
        # extract datasize
        selected_data = msg.split("datasize\":")
        value = selected_data[1].split(',')[0]
        print("_datasize = "+value)
        _datasize = int(value)
        # extract feedback
        selected_data = msg.split("feedback\":")
        value = selected_data[1].split(',')[0]
        print("_feedback value = "+value)
        _feedback = int(value)
        # extract T
        selected_data = msg.split("T\":")
        value = selected_data[1].split(',')[0]
        print("_T = "+value)
        _T = float(value)
        # extract kp
        selected_data = msg.split("kp\":")
        value = selected_data[1].split(',')[0]
        print("_kp = "+value)
        _kp = float(value)                
        # extract ki
        selected_data = msg.split("ki\":")
        value = selected_data[1].split(',')[0]
        print("_ki = "+value)
        _ki = float(value)                
        # extract kd
        selected_data = msg.split("kd\":")
        value = selected_data[1].split(',')[0]
        print("_kd = "+value)
        _kd = float(value)                
        # extract kt
        selected_data = msg.split("kt\":")
        value = selected_data[1].split(',')[0]
        print("_kt = "+value)
        _kt = float(value)                
        # extract wp
        selected_data = msg.split("wp\":")
        value = selected_data[1].split(',')[0]
        print("_wp = "+value)
        _wp = float(value)                
        # extract wd
        selected_data = msg.split("wd\":")
        value = selected_data[1].split(',')[0]
        print("_wd = "+value)
        _wd = float(value)                
        # extract N
        selected_data = msg.split("N\":")
        value = selected_data[1].split(',')[0]
        print("_N = "+value)
        _N = float(value)
        
        if initparm or update: # update parameters
            controller = _controller
            plantsim = _plantsim
            datasize = _datasize
            feedback = _feedback
            T = _T
            kp = _kp
            ki = _ki
            kd = _kd
            kt = _kt
            wp = _wp
            wd = _wd
            N = _N
            print("Parameter updated with values from shadow")
            update_dashboard()
            parms_update = 0  # reset parms_update

if initparm:
    load_parms_from_shadow() # load parameters 
# -----------------------------------
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

# now can choose between PID and output-feedback controller (OFC)
# controller = 0, 1, 2 (PID, OFC, SFC)
controller = 0
plantsim = 0 # plant simulation mode

# global variable for LSID
lsid = 0  # flag
PRBSVal = 1 # PRBS value, unit in volt
feedin = 0
bvec =[0]*13 # unit delay output
bvec[1] = bvec[2] = bvec[6] = bvec[7] = bvec[10] = bvec[12] = 1

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

# --- global variables for output-feedback controller ----
# integrator (this may be used later for integrator + sfb shcieme
def integrator_coeff_compute():
    return 0.5*T

a_int = integrator_coeff_compute()

# integrator states
u0_int = 0.0 # output
u1_int = 0.0 # past-output
e0_int = 0.0 # input
e1_int = 0.0 # past-input

def integrator(inp):  # integrator function
    global u0_int, u1_int, e0_int, e1_int
    e1_int = e0_int
    e0_int = inp
    u1_int = u0_int
    u0_int = u1_int + a_int*(e0_int + e1_int)
    return u0_int
    
def leadlag_coeff_compute(a,b): # a = zero, b = pole
    alpha1 = 2+a*T
    alpha2 = a*T - 2
    beta1 = 2+b*T
    beta2 = b*T-2
    gamma1 = -beta2/beta1
    gamma2 = alpha1/beta1
    gamma3 = alpha2/beta1
    return gamma1, gamma2, gamma3


K_ofc = 250  # output-feedback controller gain 
ll_z = 1 # lead-lag zero
ll_p = 700 # lead-lag pole
g_1, g_2, g_3 = leadlag_coeff_compute(ll_z, ll_p)

# lead-lag states
u0_ll = 0.0 # output
u1_ll = 0.0 # past-output
e0_ll = 0.0 # input
e1_ll = 0.0 # past-input

def leadlag(inp):  # lead-lag compute
    global u0_ll, u1_ll, e0_ll, e1_ll
    e1_ll = e0_ll
    e0_ll = inp
    u1_ll = u0_ll
    u0_ll = g_1*u1_ll + g_2*e0_ll + g_3*e1_ll 
    return u0_ll

# ----- state feedback controller--------
# ---- state measurement -----
# pins used for state feedback is
# x1 = 33, x2 = 32, x3(y) = 39

x_1 = ADC(Pin(33))
x_1.atten(ADC.ATTN_11DB)
x_2 = ADC(Pin(32))
x_2.atten(ADC.ATTN_11DB)



# ---- variables for autotuning ----
autotune = 0  # turn autotune off/on
#relay_a = 0.0 # amplitude of output oscillation
relay_d = 1.0  # relay output level
cycle_cnts = 0  # count oscillation cycles before measuring Tu
at_ready2measure = False  # flag to starts measuring osc magnitude and period
at_time = 0.0  # time since autotuning starts
Tu = 0.0 # ultimate period
Ku = 0.0 # ultimate gain
at_x_old = at_x_new = 0.0  # time output crosses zero (or UMID)
y_max = 0.0  # keep maximum y to compute at_a

r = 0.5  # ref. cmd
u = r   # plant input
ulim = r
u_pwm = u*v2pwm
y = 0.0  # plant output
y_old = 0.0  # previous plant output


datasize = 200  # number of points in output capture
capture = 0  # capture mode 0 = off, 1 = on
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
            if controller==2:  # state feedback uses ulim for stability 
                u_states[2*k+1] = ulim
            else:                
                u_states[2*k+1] = u
        else:
            u_states[2*k+1] = y_states[2*k-1]
        y_states[2*k+1] = (1/a)*(-b*y_states[2*k]+T*(u_states[2*k+1]+u_states[2*k]))
    return y_states[5], y_states[3], y_states[1] # changed for state-feedback controller

# def reset_lag3_states():
#     global u_states, y_states
#     for k in range(6):
#         u_states[k] = 0.0
#         y_states[k] = 0.0

# --- Functions for autotuning mode ----
# function to limit output between UMIN, UMAX and add offset UMID
def limit_and_offset(u):
    u_lim = u
    if u > UMID:
        u_lim = UMID         # limit u to UMID
    elif u < -UMID:
        u_lim = -UMID         # limit u to -UMID
    u_lim += UMID
    return u_lim

def relay(y,d):  # implement relay device
    if plantsim:
        if y>0:
            rout = -d
        else:
            rout = d
    else:
        if y>UMID:
            rout = -d
        else:
            rout = d        
    return rout

# measure oscillation amplitude period after 3 cycles
def measure_amp_period(): 
    global at_ready2measure,cycle_cnts,at_time,at_x_old,at_x_new,\
           Ku,Tu,y,y_max, autotune, prompt
    at_time+= T  # increase time during autotuning
    if cycle_cnts>2:
        #print('y = '+str(y))
        #print('y_max = '+str(y_max))
       
        if y > y_max: # keep maximum value
            y_max = y
    if plantsim:
        if y_old < 0.0 and y >= 0.0: # crossing zero detected
            at_x_old = at_x_new
            at_x_new = at_time
            cycle_cnts += 1  # cound osc cycle
            if not capture_flag:
                print("Oscillation cycle : "+str(cycle_cnts))
    else:
        if y_old < UMID and y >= UMID: # crossing UMID detected
            at_x_old = at_x_new
            at_x_new = at_time
            cycle_cnts += 1  # cound osc cycle
            if not capture_flag:
                print("Oscillation cycle : "+str(cycle_cnts))
    if cycle_cnts == 4:
        at_ready2measure = True
    if at_ready2measure:
        Tu = at_x_new - at_x_old
        if not capture_flag:
            print("Tu = "+str(Tu))
        if plantsim:
            at_a = y_max
        else:
            at_a = y_max - UMID
        Ku = 7*(4*relay_d)/(22*at_a)
        if not capture_flag:
            # print("a = "+str(at_a))
            print("Ku = "+ str(Ku))
            print("Autotuning ends.")
        autotune = 0
        update_dashboard()
        at_pid_adjust()
        prompt=True
        if not capture_flag and not online:
            sys.stdout.write('\nEnter command : ') # show prompt

def at_pid_adjust():
    global kp, ki, kd, Ku, Tu
    if not capture_flag:
        print("Updating PID parameters with Ku and Tu ")
    kp = 0.6*Ku
    ki = 1.2*Ku/Tu
    kd = 0.075*Ku*Tu
    if not capture_flag:
        print("kp = " + str(kp))
        print("ki = " + str(ki))
        print("kd = " + str(kd))
    PID_update()

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
    u_lim = limit_and_offset(u)
    return u_lim

# output-feedback controller function
def OFC_controller(r,y):
    x1 = leadlag(r-y)
    x2 = integrator(x1)
    return K_ofc*x2

# state-feedback controller
K_sfic = [100.5, 152.25, 371.25]  # state feedback gain
#ff_gain = 324  # feedforward gain
Ki_sfic = 350 

def SFIC_controller(r,x1,x2,x3):   
    u = Ki_sfic*integrator(r-x3) -K_sfic[0]*x1 -K_sfic[1]*x2 - K_sfic[2]*x3
    return u
    


# PRBS generator function
def PRBS_generator():
    global bvec, feedin, PRBSVal
    feedin = bvec[0] ^ (bvec[2]^(bvec[3]^ bvec[12]))
    for j in range(12,1,-1):  
        bvec[j] = bvec[j-1]
        #print(bvec[j])
    bvec[1] = feedin
    u_out = PRBSVal*(2*bvec[12] - 1)
    return u_out

def timer_isr(event):
    global a,b,T,u,ulim, u_states, y_states, capture_flag,t_data, \
           data_idx,datasize, prompt, y, y_old, lsid
    led.value(not led.value())
    toggle_if_pressed()
    #y_adc = y_in.read()  # read from ADC
    #y1 = y_adc*adc2v  # convert to volt
    y_old = y
    if plantsim:
        y,x2,x1 = lag3(a,b,T,u, u_states, y_states) # simulation. in volt
    else:
        y = y_in.read_uv()*1e-6 # from electronics
        x1 = x_1.read_uv()*1e-6
        x2 = x_2.read_uv()*1e-6
        
    y2rgb() # just lid RGB LED. Not discussed in book.
    if feedback:
        if controller==0: # PID selected
            if autotune:
                u = relay(y,relay_d)
                ulim = limit_and_offset(u)
                measure_amp_period()
            else:
                ulim = PID_controller(r,y)
        elif controller==1: # output-feedback controller
            u = OFC_controller(r,y)
            ulim = limit_and_offset(u)
        elif controller==2: # state-feedback controller
            u = SFIC_controller(r,x1,x2,y)
            ulim = limit_and_offset(u)
    else:
        if lsid: # LSID mode
            u = PRBS_generator()
            ulim = limit_and_offset(u)
        else:    # open-loop 
            u = r
            ulim = u
    u_pwm = int(ulim*v2pwm) # v2pwm = int(PWMMAX/3.3)
    pwm_out.duty(u_pwm)  # send output via PWM pin 16
    u_dac = int(ulim*v2dac) # v2dac = int(DACMAX/3.3)
    dac_out.write(u_dac) # also send output to DAC1
    
    if capture_flag:
        if controller==2:  # state-feedback controller
            print("[{},{},{},{},{},{},{}],".format(round(t_data,2),r,y,ulim,u,x1,x2))                    
        else:
            print("[{},{},{},{},{}],".format(round(t_data,2),r,y,ulim,u))        
        t_data+=T
        data_idx+=1
        if data_idx==datasize:
            data_idx = 0
            t_data = 0
            capture_flag = False  # stop capture data 
            print("])")  # end of np.array([ command
            prompt=True  # prompt on
            if lsid:
                print("LSID ends.")
                lsid = 0 # LSID mode ends
                update_dashboard()

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
           kp,ki,kd,kt,wp,wd,N, autotune, y, y_max, at_ready2measure, at_time,\
           at_x_old, at_x_new, cycle_cnts, lsid,a_int,g_1,g_2,g_3, controller
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
            update_dashboard()
    elif cmdstr.lower() == "capture":
        if noparm==1:
            print(capture)
        else:            
            capture = int(parmstr)
            if capture > 1:
                capture = 1
            elif capture < 0:
                capture = 0
            update_dashboard()
    elif cmdstr.lower() == "datasize":
        if noparm==1:
            print(datasize)
        else:            
            datasize = int(parmstr)
            if datasize > 2000: # set maximum datasize
                datasize = 2000
            elif datasize < 10: # minimum datasize
                datasize = 10
            update_dashboard()    
    elif cmdstr.lower() == "feedback":
        if noparm==1:
            print(feedback)
        else:            
            feedback = int(parmstr)
            if feedback > 1:
                feedback = 1
            elif feedback < 0:
                feedback = 0
            update_dashboard()
    # addeed in ofc_pid_lag3.py June 2023
    elif cmdstr.lower() == "controller":
        if noparm==1:
            print(controller)
        else:            
            parm_tmp = int(parmstr)
            if parm_tmp > 2 or parm_tmp < 0:
                print("Bad controller choice")
            else:
                controller = parm_tmp
                update_dashboard()

    elif cmdstr.lower() == "t":
        if noparm==1:
            print(T)
        else:            
            T = float(parmstr)
            if T > 1: # set maximum T
                T = 1
            elif T < 0.01: # minimum T
                T = 0.01
            T_ms = int(1000*T)
            a,b = lag3_init(T)
            PID_update()
            update_dashboard()
            # recompute coefficients for output feedback controller
            a_int = integrator_coeff_compute()
            g_1, g_2, g_3 = leadlag_coeff_compute(ll_z, ll_p)

            
            timer.init(period=T_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)
    elif cmdstr.lower() == "kp":
        if noparm==1:
            print(kp)
        else:            
            kp = float(parmstr)
            if kp > 100: # set maximum kp
                kp = 100
            elif kp < 0: # minimum kp
                kp = 0
            PID_update()
            update_dashboard()

    elif cmdstr.lower() == "ki":
        if noparm==1:
            print(ki)
        else:            
            ki = float(parmstr)
            if ki > 100: # set maximum ki
                ki = 100
            elif ki < 0: # minimum ki
                ki = 0
            PID_update()
            update_dashboard()

    elif cmdstr.lower() == "kd":
        if noparm==1:
            print(kd)
        else:            
            kd = float(parmstr)
            if kd > 100: # set maximum kd
                kd = 100
            elif kd < 0: # minimum kd
                kd = 0
            PID_update()
            update_dashboard()

    elif cmdstr.lower() == "kt":
        if noparm==1:
            print(kt)
        else:            
            kt = float(parmstr)
            if kt > 100: # set maximum kt
                kt = 100
            elif kt < 0: # minimum kt
                kt = 0
            PID_update()
            update_dashboard()

    elif cmdstr.lower() == "wp":
        if noparm==1:
            print(wp)
        else:            
            wp = float(parmstr)
            if wp > 10: # set maximum wp
                wp = 10
            elif wp < 0: # minimum wp
                wp = 0
            PID_update()
            update_dashboard()

    elif cmdstr.lower() == "wd":
        if noparm==1:
            print(wd)
        else:            
            wd = float(parmstr)
            if wd > 10: # set maximum wd
                wd = 10
            elif wd < 0: # minimum wd
                wd = 0
            PID_update()
            update_dashboard()

    elif cmdstr.lower() == "n":
        if noparm==1:
            print(N)
        else:            
            N = float(parmstr)
            if N > 200: # set maximum N
                N = 200
            elif N < 2: # minimum N
                N = 2
            PID_update()
            update_dashboard()        
    elif cmdstr.lower() == "autotune":
        if not autotune: # prohibits if already running
            prompt=False
            if not capture:
                print("Autotuning starts ...")
            autotune = 1
            update_dashboard()
            at_ready2measure = False # clear flag
            at_time = 0.0
            at_x_old = at_x_new = 0.0
            y_max = 0.0
            cycle_cnts = 0
            if capture:
                print("datamat = np.array([")  # head of data matrix
                prompt=False # turn prompt off 
                capture_flag = True
    elif cmdstr.lower() == "y":
        print(y)
    elif cmdstr.lower() == "ku": # ultimate gain from autotune
        print(Ku)
    elif cmdstr.lower() == "tu": # ultimate period from autotune
        print(Tu)
    elif cmdstr.lower() == "lsid":
        if not lsid: # prohibits if already running
            feedback = 0 # in open-loop mode
            lsid = 1
            update_dashboard()
            # outputting data
            print("datamat = np.array([")  # head of data matrix
            prompt=False # turn prompt off 
            capture_flag = True # output data
    elif cmdstr.lower() == "dbupdate": # update dashboard
        update_dashboard()
    # ----added in iot_controller3.py
    # Command to change parameter initialization source
    elif cmdstr.lower() == "initparm":
        if noparm==1:
            print(initparm)
        else:            
            initparm = int(parmstr)
            if initparm > 1:
                initparm = 1
            elif initparm < 0:
                initparm = 0
            initparm_data['initparm'] = initparm
            publish_str = ujson.dumps({"data": initparm_data})
            client.publish("@shadow/data/update", publish_str)        
    # save all parameters to shadow
    elif cmdstr.lower() == "saveparms":
        save_parms_to_shadow()
    # load parameters from shadow
    elif cmdstr.lower() == "loadparms":
        parms_update = 1
        load_parms_from_shadow()
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
update_dashboard()

def toggle_if_pressed(): # toggle online flag if button pressed
    global button_state, online
    if button_state != button.value():
        sleep_ms(100)  # debounce
        button_state = button.value()
        if button_state:
            online = not online
            if online:
                print("\nswitch to online mode")
            else:
                print("\nSwitch to offline mode")


while True:
    if online:
        time_current = time.ticks_ms()
        publish_delta = time_current - time_prev
        if publish_delta>PUBLISH_PERIOD: # publish interval must be larger than PUBLISH_PERIOD
            time_prev = time_current
            client.check_msg()
            
            shadow_data['r'] = r
            shadow_data['y'] = y
            shadow_data['u'] = ulim
            
            publish_str = ujson.dumps({"data": shadow_data})
            # print(publish_str)
            client.publish("@shadow/data/update", publish_str)
            # added in iot_controller4.py
            sleep_ms(1000)
            csv_str = "{},{},{}".format(r,y,ulim)
            client.publish("@msg/shadowdata", csv_str)
            # added in iot_controller3.py
            # receive data from shadow if requested
            if (private_request_flag):
                split_parms(update = parms_update)
        
    else:
        user_input()

client.disconnect()