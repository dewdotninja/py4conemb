# din_test.py
# dew.ninja June 2023
# test digital input with switch IO0 on NodeMCU-32S

import math
import time
from time import sleep_ms
from machine import Pin, PWM

obled = PWM(Pin(2))  # on-board led
button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch

PWMMAX = 1023
PWMMID = 511
PWMMIN = 0

# genchirp() check button status
# and generate chirp signal if pressed

def genchirp():  # check button status and generate chirp signal if pressed
    stepnum = 10
    sinstep = 6.28/stepnum
    sinangle = 0
    dt = 30
    cyclecomplete = False
    if not button.value():
        sleep_ms(100)  # debounce
        while not button.value(): # blocked if switch pressed
            sinangle+=sinstep
            if sinangle>6.28:
                cyclecomplete = True
                sinangle = 0
            chirpout = int(PWMMID*0.5*(math.sin(sinangle)+1))
            if chirpout> PWMMAX:
                chirpout = PWMMAX
            if chirpout < PWMMIN:
                chirpout = PWMMIN                
            obled.duty(chirpout)
            if cyclecomplete:
                dt-=1
                cyclecomplete = False
            if dt<2:
                dt = 30
            sleep_ms(dt)
while True:
    genchirp()  # press IO0 sw to generate chirp signal
            
