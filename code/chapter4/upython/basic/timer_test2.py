# timer_test2.py
# dew.ninja June 2021
# use timer to blink on-board led
# write callback function

import machine
from machine import Pin, Timer        #importing pin, and timer class
led= Pin(2, Pin.OUT)              # GPIO2 as led output
            
timer1=Timer(1)

def timer_isr(event): 
    led.value(not led.value())
    print("LED value = "+str(led.value()))

#initializing the timer and blink LED
timer1.init(period=1000, mode=Timer.PERIODIC, callback=timer_isr)
