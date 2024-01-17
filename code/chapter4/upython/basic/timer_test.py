# timer_test.py
# dew.ninja June 2021
# use timer to blink on-board led

import machine
from machine import Pin, Timer        #importing pin, and timer class
led= Pin(2, Pin.OUT)              # GPIO2 as led output

led.value(0)              #LED is off
timer=Timer(-1)

timer.init(period=1000, mode=Timer.PERIODIC, callback=lambda t:led.value(not led.value()))   #initializing the timer