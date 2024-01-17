#timer_2threads_ledcmd.py
# test multitasking with thread
# both main loop and thread executes same task
# that takes 414 ms to finish

import time
import _thread
from machine import Pin, Timer
import sys

led = Pin(2, Pin.OUT)  # on-board led
rled = Pin(19, Pin.OUT)
gled = Pin(18, Pin.OUT)
bled = Pin(17, Pin.OUT)

def task1():
    rled.value(not rled.value())

def task2():
    gled.value(not gled.value())
    time.sleep(0.5)
    
def do_task2_forever():
    while True:
        task2()

# start a new thread
_thread.start_new_thread(do_task2_forever,())

timer1 = Timer(1)

def timer_isr(event):
    task1()

timer1.init(period=100, mode=Timer.PERIODIC, callback=timer_isr)

def splitCmd(userstr):
    result = userstr.find("=")
    if result == -1:
        noparm = 1
        cmdstr = userstr.strip()
        parmstr = '' # not used
    else:
        noparm = 0
        splitstr = userstr.split("=")
        cmdstr = splitstr[0].strip()
        parmstr = splitstr[1].strip()
    return cmdstr, parmstr, noparm

def cmdInt(userstr):
    
    cmdstr, parmstr, noparm = splitCmd(userstr)

    if cmdstr.lower() == "btoggle":
        # toggle blue LED
        bled.value(not bled.value())
        if bled.value():
            print("Blue LED turned on")
        else:
            print("Blue LED turned off")
    else:
        print("Invalid command")
        
def user_input():
    sys.stdout.write('\nEnter command : ')
    user_str = sys.stdin.readline()
    newline_char = sys.stdin.readline()
    cmdInt(user_str)


while True:
    user_input()
