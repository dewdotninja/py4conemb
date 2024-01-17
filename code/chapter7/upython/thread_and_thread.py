#thread_and_thread.py
# test multitasking with thread
# both threads executes same task
# that takes 414 ms to finish

import time
import _thread
#from machine import Pin

#button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch
#led = Pin(2, Pin.OUT)  # on-board led
#button_toggle = 0  # 

def task1():
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from thread 1 with period " + str(dt))

def task2():
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("Task executed from thread 2 with period " + str(dt))

def do_task1_forever():
    while True:
        task1()
        
def do_task2_forever():
    while True:
        task2()

# start a new thread
_thread.start_new_thread(do_task1_forever,())

# start a new thread
_thread.start_new_thread(do_task2_forever,())


# main loop
while True:
    pass