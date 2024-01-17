# measure_task_period.py
#
# Create a nested loop task and measure period
# set m = 300 yields loop period of 414 millisecs 

import time

# t_start = 0
# t_end = 0
# dt = 0  # loop period

def nested_loop_task():
#     global dt
    t_start =time.ticks_ms()
    m = 300
    for i in range(m):
        for j in range(m):
            y = i+j
    t_end = time.ticks_ms()
    dt = t_end - t_start
    print("loop period = " + str(dt)+ " millisecs")

nested_loop_task()

