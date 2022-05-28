"""
For Thesis Completion ...
Author: Andre' Bamba
Version: 1
Program: A water quality monitoring and control program using ESP32 IoT connection with RPI controller
Description: ESP832 controller monitors water quality and controls actuation features such as irrigation and dosing control.
             Control and monitoring communicated with RPi through MQTT protocol for IoT automation project.
             Makes use of freeRTOS for task distribution using two cores (ESP32 is dual core), one for monitoring and other for control.

For monitoring:
    Water Temperature: DS18B20    GPIO 5
    TDS/EC: Grove TDS Meter       GPIO 33
    Water Flow: YF-S201           GPIO 32
    Water level: HC SR04          GPIO 25 26        
    Water pH: SEN0169V2 pH meter  GPIO ?  

For control:
    4 x peristaltic pumps         GPIO 27 14 12 13
    2 chnl relay and soln valve   GPIO 16 17 18

  - Creating separate functions for data acquisition and nutrient dosing
  - Nutrient dosing algorithm (for remote automation)                         
  - pH monitoring/control functions

Subscribed topics in ESP32
    topic_pub_temp "hydro/water/temp"
    topic_pub_level "hydro/water/waterlevel"
    topic_pub_tds "hydro/water/tds"
    topic_pub_flowrate "hydro/water/flowrate"

Published topics to ESP32
    topic_sub_irrigation "hydro/water/irrigation"
    topic_sub_nutdos "hydro/water/nutdos"

"""
# Main Program -----------------------------------------------------------------------
import paho.mqtt.client as mqtt
import time


# Libraries for DS18B20 --------------------------------------------------------------
"""
from w1thermsensor import W1ThermSensor
"""

# Libraries for Grove TDS Meter ------------------------------------------------------
"""
import math
import sys
from grove.adc import ADC
"""

# Libraries for YF-S201 --------------------------------------------------------------


# Libraries for HC SR04 --------------------------------------------------------------


# Libraries for SEN0169V2 pH meter ---------------------------------------------------


MQTT_SERVER = '192.168.1.10'
MQTT_UN = 'hydropo'
MQTT_PW = 'hydropo'
MQTT_TOPIC = 'hydro/+/+'


#The callback for when the client receives a CONNACK response from the server
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Client is connected.")
        client.subscribe(MQTT_TOPIC)
    else:
        print('Client is not connected. Result code is ' + str(rc))
    

#The callback for when a PUBLISH message is received
def on_message(client, userdata, msg):
    #t = [float(x) for x in msg.payload.decode("utf-8")]
    print("Message received" + str(msg.payload.decode("utf-8")))
    print("Topic" + str(msg.topic))


mqtt_client = mqtt.Client("client_hydro")
mqtt_client.username_pw_set(MQTT_UN, MQTT_PW)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
#mqtt_client.connect(MQTT_SERVER, 1883)



# Configure [Insert sensor]

try:
    while True:

        # Publish MQTT sensor data
        try:
            mqtt_client.connect()
            try:
                client.publish()
                print("Published MQTT sensor data.")

                client.publish()
                print("Published MQTT binary sensor data.")

            except OSError:
                print("Failed to publish MQTT sensor data.")

            client.disconnect()
        except OSError:
            print("Failed to connect MQTT client")

            sleep(10)
except KeyboardInterrupt:
    print("\nCtrl-C pressed. Cleaning up serial port and exiting...")
finally:
    port.deinit()






