# rgbfancy_netpie.py
# dew.ninja Oct 2022
# OOP implementation of RGB LED on LAG3 board.
# add NETPIE functionality

import machine
import time
import random
from machine import Pin, PWM, Timer
import sys

PWMMAX = 1023

class RGB:
    def __init__(self,rpin,gpin,bpin):
        self.rled = PWM(Pin(rpin),freq=100)
        self.gled = PWM(Pin(gpin),freq=100)
        self.bled = PWM(Pin(bpin),freq=100)
        self.rval = 0
        self.gval = 0
        self.bval = 0
    
    def lid(self,rval,gval,bval):
        self.rval = rval
        self.gval = gval
        self.bval = bval
        self.rled.duty(self.rval)
        self.gled.duty(self.gval)
        self.bled.duty(self.bval)
        
class RGBfancy(RGB):
    def __init__(self,rpin,gpin,bpin,rate):
        super().__init__(rpin,gpin,bpin)
        self.rate = rate
        self.period = 1/rate
    def getperiod(self):
        return self.period
    def getrate(self):
        return self.rate
    def setrate(self,rate):
        self.rate = rate
        self.period = 1/rate
    def party(self):
        self.lid(random.randint(0, PWMMAX),random.randint(0, PWMMAX),random.randint(0, PWMMAX))

rgb = RGBfancy(19,18,17,10)

# --- added for iot development-----
from umqtt.robust import MQTTClient
import ujson
import network

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = "7"
MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""
PUBLISH_PERIOD = 2000  # milliseconds
shadow_data = {'r': 0, 'g': 0, 'b': 0, 'rate': 0}
#time_current = 0  # variables to control publishing period
time_prev = 0 # variables to control publishing period
timer=Timer(1)

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
    print((topic, msg))
    if topic == b'@msg/cmd':
        rcvdstrs = str(msg).split("'") # get rid of b'
        rcvdstr = rcvdstrs[1]
        cmdInt(rcvdstr, rgb)

online = True
if online:
    wifi_connect()  # connect to WiFi network
    init_client()

# ----------command intepreter ---------
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


def cmdInt(userstr, rgb):
    cmdstr, parmstr, noparm = splitCmd(userstr)    
    if cmdstr.lower() == "rate":
        if noparm==1:
            print(rgb.getrate())
        else:            
            rate = float(parmstr)
            if rate > 10: # set maximum rate
                rate = 10
            elif rate < 0.1: # minimum rate
                rate = 0.1
            rgb.setrate(rate)
            period_ms = int(1000*rgb.getperiod())            
            timer.init(period=period_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)
    else:
        print("Invalid command")

prompt = True
def user_input():
    if prompt:
        sys.stdout.write('\nEnter command : ')
    user_str = sys.stdin.readline()
    newline_char = sys.stdin.readline()
    cmdInt(user_str,rgb)

def timer_isr(event):
    rgb.party()

period_ms = int(1000*rgb.getperiod())
timer.init(period=period_ms, mode=Timer.PERIODIC,
                           callback=timer_isr)

while True:
    if online:
        time_current = time.ticks_ms()
        publish_delta = time_current - time_prev
        if publish_delta>PUBLISH_PERIOD: # publish interval must be larger than PUBLISH_PERIOD
            time_prev = time_current
            client.check_msg()
            
            shadow_data['r'] = rgb.rval
            shadow_data['g'] = rgb.gval
            shadow_data['b'] = rgb.bval
            shadow_data['rate'] = rgb.getrate()
            
            publish_str = ujson.dumps({"data": shadow_data})
            print(publish_str)
            client.publish("@shadow/data/update", publish_str)
            # added in iot_controller4.py
        
    else:
        user_input()

client.disconnect()


        