# nested_loop_timer.py

# test multitasking with timer
# both main loop and timer executes same task
# that takes 414 ms to finish

import time
from machine import Pin, Timer

button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch
led = Pin(2, Pin.OUT)  # on-board led

timer1 = Timer(1)

def timer_isr(event):
    nested_loop_task("timer")
    

def nested_loop_task(caller_id):
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from "+caller_id+" with period "+str(dt)+" ms")

timer1.init(period=430, mode=Timer.PERIODIC, callback=timer_isr)

# main loop
while True:
    nested_loop_task("main loop")
    if not button.value():
        time.sleep_ms(100)
        print("Main loop blocked")
        while not button.value():
            led.value(not led.value()) # blink onboard led
            time.sleep_ms(100)
    
