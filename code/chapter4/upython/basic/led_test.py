# led_test.py
# test on-board led on new ESP32 board

from machine import Pin
from utime import sleep_ms

led = Pin(2, Pin.OUT)

while True:
    led.on()
    print("Hello")
    sleep_ms(500)