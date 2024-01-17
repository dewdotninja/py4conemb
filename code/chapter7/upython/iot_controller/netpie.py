# netpie.py
# dew.ninja   Oct 2022
# class for NETPIE2020 communication
# updated: Mar 23
#  add wrapper functions such as publish(), disconnect() 

import network
import time
from umqtt.robust import MQTTClient

class NETPIE:
    def __init__(self, wifi_ssid, wifi_pwd, mqtt_client, mqtt_user, mqtt_pwd):
        self.wifi_ssid = wifi_ssid
        self.wifi_pwd = wifi_pwd
        self.mqtt_broker = "broker.netpie.io"
        self.mqtt_client = mqtt_client
        self.mqtt_user = mqtt_user
        self.mqtt_pwd = mqtt_pwd
        
        # command delay is required for node-red implementation
        self.CMD_DELAY = 1000
        self.cmdtime_prev = 0
        self.cmd_received = False
        self.cmdstr = []
        self.private_msg = ''
        
        self.wifi_connect()
        self.init_client()
        
    def wifi_connect(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            print('connecting to network...')
            wlan.connect(self.wifi_ssid, self.wifi_pwd)
            while not wlan.isconnected():
                pass
        print('network config:', wlan.ifconfig())

    def init_client(self):
        print("Trying to connect to mqtt broker.")
        try:
            self.client = MQTTClient(self.mqtt_client,self.mqtt_broker, port=1883, user=self.mqtt_user,
                                password=self.mqtt_pwd)
            self.client.connect()
            print("Connected to ",self.mqtt_broker)
            self.topic_sub = b"@msg/cmd"
            print("Subscribed to ",self.topic_sub)
            self.client.set_callback(self.sub_cb)
            self.client.subscribe(self.topic_sub)
            self.topic_private_sub = b"@private/#"
            print("Subscribed to ",self.topic_private_sub)
            self.client.subscribe(self.topic_private_sub)
            
        except:
            print("Problem initializing mqtt.") 

    def sub_cb(self,topic, msg):
        print((topic, msg))
        if topic == b'@msg/cmd':
            rcvdstrs = str(msg).split("'") # get rid of b'
            rcvdstr = rcvdstrs[1]
            # Update: use filter node.
            #this delay is no longer needed for nodered implementation
            #cmdtime_current = time.ticks_ms()
            #delta_cmdtime = cmdtime_current - self.cmdtime_prev
            #if delta_cmdtime > self.CMD_DELAY:
            #    self.cmdtime_prev = cmdtime_current
            self.cmd_received = True  # flag to indicate command message is received
            self.cmdstr = rcvdstr
                #cmdInt(rcvdstr)
        elif topic == b'@private/shadow/data/get/response':
            # print(msg)
            self.private_msg = str(msg)

    def retrieve_cmd(self): # check whether a command is received
        if self.cmd_received:
            self.cmd_received = False
            return self.cmdstr
    # load parameters from shadow. Added Mar 23
    def get_private_message(self):
        return self.private_msg
    # wrapper functions added  Mar 23
    def publish(self,topic, msg):
        self.client.publish(topic,msg)
    
    def check_msg(self):
        self.client.check_msg()
        
    def disconnect(self):
        self.client.disconnect()