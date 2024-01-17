# timer_test1.py
# dew.ninja June 2021
# use timer to blink on-board led

import machine
from machine import Pin, Timer        #importing pin, and timer class
led= Pin(2, Pin.OUT)              # GPIO2 as LED output

timer=Timer(-1)

#initializing the timer and blink LED
timer.init(period=1000, mode=Timer.PERIODIC,
           callback=lambda t:led.value(not led.value()))   