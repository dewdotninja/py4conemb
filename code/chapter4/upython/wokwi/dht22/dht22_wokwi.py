#DHT22_wokwi.py
# dew.ninja  Sep 2023

import dht
import machine
from utime import sleep_ms
d = dht.DHT22(machine.Pin(4))
while True:
    d.measure()
    temperature=d.temperature() 
    humidity=d.humidity()
    print("temperature = ",temperature)
    print("humidity = ",humidity)
    sleep_ms(2000)