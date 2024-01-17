#rgb_basic.py

import machine
from machine import Pin, PWM

PWMMAX = 1023

rled = PWM(Pin(19))  # red led
gled = PWM(Pin(18))  # green led
bled = PWM(Pin(17))  # blue led

def lidRGBled(rval, gval, bval):
    rled.duty(rval)
    gled.duty(gval)
    bled.duty(bval)

# adjust RGB value to change color
lidRGBled(0,0, PWMMAX)