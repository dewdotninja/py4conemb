# rgb.py
# dew.ninja Oct 2022
# OOP implementation of RGB LED on LAG3 board.

import machine
from machine import Pin, PWM

PWMMAX = 1023

class RGB:
    def __init__(self,rpin,gpin,bpin):
        self.rled = PWM(Pin(rpin),freq=100)
        self.gled = PWM(Pin(gpin),freq=100)
        self.bled = PWM(Pin(bpin),freq=100)
        self.rval = 0
        self.gval = 0
        self.bval = 0
    
    def lid(self,rval,gval,bval):
        self.rval = rval
        self.gval = gval
        self.bval = bval
        self.rled.duty(self.rval)
        self.gled.duty(self.gval)
        self.bled.duty(self.bval)
        


rgb = RGB(19,18,17)
rgb.lid(PWMMAX,0,0)
        