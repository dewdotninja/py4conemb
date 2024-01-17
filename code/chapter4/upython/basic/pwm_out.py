#pwm_out.py
# dew.ninja  June 2021
# test PWM output to pin 2 (on-board LED)
# The led brightness should vary linearly between off and
# max brightness

from machine import Pin, PWM
from utime import sleep_ms

PWMMAX = 1023
pwm2 = PWM(Pin(2)) # assign output to pin 2
pwm2.freq(2000)  # set frequency

pwmval = 0  # PWM value
step = 10
direction = 0  # 0 = increase, 1 = decrease

while True:
    if direction==0:
        pwmval += step
    else:
        pwmval -= step
    if pwmval>PWMMAX:
        pwmval = PWMMAX
        direction = 1  # decrease
    elif pwmval<0:
        pwmval = 0
        direction = 0  # increase
    pwm2.duty(pwmval)  # send output
    sleep_ms(20)
    