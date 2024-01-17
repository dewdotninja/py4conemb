# cmdtest2_ww.py
# dew.ninja Sep 2023
# for Wokwi simulation

from machine import Pin, Timer
#import sys
from utime import sleep_ms

led = Pin(2, Pin.OUT)
led_period = 1000  # default
toggle_mode = 1
led_status = 0

timer=Timer(1)
def timer_isr(event): 
    led.value(not led.value())

timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)

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
    global led_status, toggle_mode, led_period, timer,led
    cmdstr, parmstr, noparm = splitCmd(userstr)

    if cmdstr.lower() == "led":
        if noparm==1:
            led_status = led.value()
            print("Current led status = {}".format(led_status))
        else:            
            led_status = int(parmstr)
            if led_status > 1:
                led_status = 1
            elif led_status < 0:
                led_status = 0
            if led_status:
                led.on()  # turn on led
            else:
                led.off()
    elif cmdstr.lower() == "toggle":
        if noparm==1:
            print("Current toggle mode = {}".format(toggle_mode))
        else:            
            toggle_mode = int(parmstr)
            if toggle_mode >= 1:
                toggle_mode = 1
                timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)
            elif toggle_mode <= 0:
                toggle_mode = 0
                timer.deinit()
    elif cmdstr.lower() == "period":
        if noparm==1:
            print("Current led period = {}".format(led_period))
        else:            
            led_period = int(parmstr)
            if led_period > 2000: # set maximum period to 2 seconds
                led_period = 2000
            elif led_period < 50: # minimum period is 50 milliseconds
                led_period = 50
            if toggle_mode: # deinit and init with new period
                timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)                
    else:
        print("Invalid command")    

def user_input():
    #sys.stdout.write('\nEnter command : ')
    print('\nEnter command : ')
    #user_str = sys.stdin.readline()
    #newline_char = sys.stdin.readline()
    user_str = input()
    cmdInt(user_str)


while True:
    user_input()

