#timer_main_thread.py
# test multitasking with thread
# both main loop and thread executes same task
# that takes 414 ms to finish

import time
import _thread
from machine import Pin, Timer

led = Pin(2, Pin.OUT)  # on-board led

def task1():
#     global dt
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from timer with period " + str(dt))

def task2():
#     global dt
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from thread with period " + str(dt))

def task3():
#     global dt
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from main with period " + str(dt))
def do_task2_forever():
    while True:
        task2()

# start a new thread
_thread.start_new_thread(do_task2_forever,())

timer1 = Timer(1)

def timer_isr(event):
    task1()

timer1.init(period=420, mode=Timer.PERIODIC, callback=timer_isr)

while True:
    task3()
