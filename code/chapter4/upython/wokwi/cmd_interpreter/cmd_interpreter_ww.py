# cmd_interpreter_ww.py
# dew.ninja  August 2023
# for Wokwi simulation. Use input() instead of sys.stdin.readline()
# Seperate command from parameter

#import sys
from utime import sleep_ms

#command interpreter function
def cmdInt(userstr):
    result = userstr.find("=")
    if result == -1:
        noparm = 1
        cmdstr = userstr.strip()
    else:
        noparm = 0
        splitstr = userstr.split("=")
        cmdstr = splitstr[0].strip()
        parmstr = splitstr[1].strip()
    print("command = " +cmdstr)
    if noparm:
        print('No parameter in command')
    else:
        print("parameter = " + parmstr)
    
def user_input():
    print('===================')
    print('Enter command : ')
    user_str = input()
    cmdInt(user_str)

while True:
    user_input()

