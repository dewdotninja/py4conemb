# cmd_interpreter.py
# dew.ninja  June 2023
# Seperate command from parameter

import sys
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
    sys.stdout.write('Enter command : ')
    user_str = sys.stdin.readline()
    newline_char = sys.stdin.readline()
    cmdInt(user_str)

while True:
    user_input()
