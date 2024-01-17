# cmdtest2_iot.py
# dew.ninja
# June 2023
# extend to iot

from machine import Pin, Timer
import sys
import time
from utime import sleep_ms

# --- added for iot development-----
from umqtt.robust import MQTTClient
import ujson
import network

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = ""
MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""
PUBLISH_PERIOD = 2000  # milliseconds
shadow_data = {'led': 0, 'toggle': 0, 'period': 0}
#time_current = 0  # variables to control publishing period
time_prev = 0 # variables to control publishing period


def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifi_ssid, wifi_pwd)
        while not wlan.isconnected():
            pass
    #print('network config:', wlan.ifconfig())

def init_client():
    global client

    print("Trying to connect to mqtt broker.")

    try:
        client = MQTTClient(MQTT_CLIENT, MQTT_BROKER, port=1883, user=MQTT_USER,
                            password=MQTT_PWD)
        client.connect()
        print("Connected to ",MQTT_BROKER)
        topic_sub = b"@msg/cmd"
        print("Subscribed to ",topic_sub)
        client.set_callback(sub_cb)
        client.subscribe(topic_sub)
        
    except:
        print("Trouble to init mqtt.") 

def sub_cb(topic, msg):
    #global cmdtime_current, cmdtime_prev
    print((topic, msg))
    if topic == b'@msg/cmd':
        rcvdstrs = str(msg).split("'") # get rid of b'
        rcvdstr = rcvdstrs[1]
        cmdInt(rcvdstr)
        # this delay is needed for nodered implementation
#         cmdtime_current = time.ticks_ms()
#         delta_cmdtime = cmdtime_current - cmdtime_prev
#         if delta_cmdtime > CMD_DELAY:
#             cmdtime_prev = cmdtime_current
#             cmdInt(rcvdstr)
online = True
if online:
    wifi_connect()  # connect to WiFi network
    init_client()

button = Pin(0,Pin.IN, Pin.PULL_UP) # on-board switch
button_state = True
# --------------------------------

led = Pin(2, Pin.OUT)
led_period = 1000  # default
toggle_mode = 1
led_status = 0



timer=Timer(1)

def timer_isr(event):
    toggle_if_pressed()            
    led.value(not led.value())

timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)

# helper function for cmdInt()
# separate command and parameter
# at "="
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

#command interpreter function
def cmdInt(userstr):
    global led_status, toggle_mode, led_period, timer,led
    cmdstr, parmstr, noparm = splitCmd(userstr)

    if cmdstr.lower() == "led":
        if noparm==1:
            led_status = led.value()
            print("Current led status = {}".format(led_status))
        else:            
            led_status = int(parmstr)
            if led_status > 1:
                led_status = 1
            elif led_status < 0:
                led_status = 0
            if led_status:
                led.on()  # turn on led
            else:
                led.off()
    elif cmdstr.lower() == "toggle":
        if noparm==1:
            print("Current toggle mode = {}".format(toggle_mode))
        else:            
            toggle_mode = int(parmstr)
            if toggle_mode >= 1:
                toggle_mode = 1
                timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)
            elif toggle_mode <= 0:
                toggle_mode = 0
                timer.deinit()
    elif cmdstr.lower() == "period":
        if noparm==1:
            print("Current led period = {}".format(led_period))
        else:            
            led_period = int(parmstr)
            if led_period > 2000: # set maximum period to 2 seconds
                led_period = 2000
            elif led_period < 50: # minimum period is 50 milliseconds
                led_period = 50
            if toggle_mode: # deinit and init with new period
                timer.init(period=led_period, mode=Timer.PERIODIC,
                           callback=timer_isr)                
    else:
        print("Invalid command")    

def user_input():
    sys.stdout.write('\nEnter command : ')
    user_str = sys.stdin.readline()
    newline_char = sys.stdin.readline()
    cmdInt(user_str)

def toggle_if_pressed(): # toggle online flag if button pressed
    global button_state, online
    if button_state != button.value():
        sleep_ms(100)  # debounce
        button_state = button.value()
        if button_state:
            online = not online
            if online:
                print("\nswitch to online mode")
            else:
                print("\nSwitch to offline mode")


while True:
    if online:
        time_current = time.ticks_ms()
        publish_delta = time_current - time_prev
        if publish_delta>PUBLISH_PERIOD: # publish interval must be larger than PUBLISH_PERIOD
            time_prev = time_current
            client.check_msg()
            
            shadow_data['led'] = led.value()
            shadow_data['toggle'] = toggle_mode
            shadow_data['period'] = led_period
            
            publish_str = ujson.dumps({"data": shadow_data})
            print(publish_str)
            client.publish("@shadow/data/update", publish_str)        
        
    else:
        user_input()
