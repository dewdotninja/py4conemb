# rgb.py
# dew.ninja Oct 2022
# OOP implementation of RGB LED on LAG3 board.

import machine
import time
import random
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
        
class RGBfancy(RGB):
    def __init__(self,rpin,gpin,bpin,rate):
        super().__init__(rpin,gpin,bpin)
        self.rate = rate
    def party(self):
        period = 1/self.rate
        while True:  
            self.lid(random.randint(0, PWMMAX),random.randint(0, PWMMAX),random.randint(0, PWMMAX))
            time.sleep(period)
    

rgb = RGBfancy(19,18,17,10)
rgb.party()

        