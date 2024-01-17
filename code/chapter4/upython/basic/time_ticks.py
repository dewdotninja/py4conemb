# time_ticks.py
# dew.ninja  June 2023
# period control using time.ticks_ms()

import time
from machine import Pin

led= Pin(2, Pin.OUT)              # GPIO2 as led output
blink_period = 1000

time_prev = 0

while True:
    time_current = time.ticks_ms()
    if time_current<time_prev:
        time_prev = time_current # prevent overflow
    if (time_current - time_prev)>blink_period:
        led.value(not led.value())  # blink on-board led
        time_prev = time_current
