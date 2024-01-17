#adc0_test.py
# dew.ninja  June 2021
# test reading from ADC0 (GPIO36)

from machine import Pin, ADC
from utime import sleep_ms

adc = ADC(Pin(36)) #ADC1_CH0
adc.atten(ADC.ATTN_11DB) # range 0 - 3.6 V
while True:
    adcval = adc.read()
    print(adcval)
    sleep_ms(1000)
    