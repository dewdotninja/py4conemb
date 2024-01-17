from machine import Pin, Timer
from utime import sleep_ms

led = Pin(2, Pin.OUT)
x = 'noname'
getname = False

timer1=Timer(1)

def timer_isr(event): 
    led.value(not led.value())
    #print("LED value = "+str(led.value()))

#initializing the timer and blink LED
timer1.init(period=1000, mode=Timer.PERIODIC, callback=timer_isr)

def user_input():
    global x, getname
    print('Enter your name : ')
    x = input()
    getname = True


#uart = UART(0)
while True:
    user_input()
    if getname:
        print('Hello, ' + x)
        getname = False
    #uart.write("Hello")
    #print(uart.read())