# cmdtest2_uart.py
# dew.ninja
# June 2023
# This version communicates via uart

from machine import Pin, Timer, UART
from utime import sleep_ms
import sys

led = Pin(2, Pin.OUT)
led_period = 50  # default
timer=Timer(1)
toggle_mode = 1
led_status = 0

def timer_isr(event): 
    led.value(not led.value())

timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)
uart = UART(0,115200,timeout=10)
uart.write("Baud rate set to 115200")

#command interpreter function
def cmdInt(userstr):
    global led_status, toggle_mode, led_period, timer,led
    result = userstr.find("=")
    if result == -1:
        noparm = 1
        cmdstr = userstr.strip()
    else:
        noparm = 0
        splitstr = userstr.split("=")
        cmdstr = splitstr[0].strip()
        parmstr = splitstr[1].strip()
    #print(cmdstr)
    #print(parmstr)
    if cmdstr.lower() == "led":
        # toggle_mode = 0  # turn off toggle mode
        
        if noparm==1:
            led_status = led.value()
            msg="Current led status = "+str(led_status)
            uart.write(msg)
            # print("Current led status = {}".format(led_status))
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
            msg="Current toggle mode = "+str(toggle_mode)
            uart.write(msg)
            #print("Current toggle mode = {}".format(toggle_mode))
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
            msg = "Current led period = "+str(led_period)
            uart.write(msg)
            #print("Current led period = {}".format(led_period))
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
        uart.write("Invalid command")    

def user_input():
    global uart
    newcmd = False
    while uart.any():        
        user_str = uart.readline()
        uart.write("Reciving")
        uart.write(user_str)
        newcmd = True
    if newcmd:
        cmdInt(user_str)

while True:
    user_input()
    rcvdstr = sys.stdin.readline()
    uart.write(str(rcvdstr))
        
    sleep_ms(1000)