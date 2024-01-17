#nested_loop_multitask.py
# test multitasking with thread
# both main loop and thread executes same task
# that takes 414 ms to finish

import time
import _thread
from machine import Pin

button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch
led = Pin(2, Pin.OUT)  # on-board led
button_toggle = 0  # 

def nested_loop_task1():
#     global dt
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from main with period " + str(dt))

def nested_loop_task2():
#     global dt
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from thread with period " + str(dt))

def do_nested_loop_forever():
    global button_toggle
    while True:
        nested_loop_task2()
        if not button.value() and button_toggle:
            time.sleep_ms(100)
            print("thread loop blocked")
            while not button.value():
                led.value(not led.value()) # blink onboard led
                time.sleep_ms(100)
            button_toggle = 0
# start a new thread
_thread.start_new_thread(do_nested_loop_forever,())

# main loop
while True:
    nested_loop_task1()
    if not button.value() and not button_toggle:
        time.sleep_ms(100)
        print("Main loop blocked")
        while not button.value():
            led.value(not led.value()) # blink onboard led
            time.sleep_ms(100)
        button_toggle = 1