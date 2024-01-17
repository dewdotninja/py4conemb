from machine import Pin
from utime import sleep_ms

led = Pin(2, Pin.OUT)

while True:
    led.on()
    print("LED is ", led.value())
    sleep_ms(500)
    led.off()
    print("LED is ", led.value())
    sleep_ms(500)
    