"""
For Thesis Completion ...
Author: Andre' Bamba
Version: 1
Program: A water quality monitoring and control program using ESP32 IoT connection with RPI controller
Description: ESP832 controller monitors water quality and controls actuation features such as irrigation and dosing control.
             Control and monitoring communicated with RPi through MQTT protocol for IoT automation project.
             Makes use of freeRTOS for task distribution using two cores (ESP32 is dual core), one for monitoring and other for control.

For monitoring:
    Water Temperature: DS18B20    GPIO _
    TDS/EC: Grove TDS Meter       GPIO _
    Water Flow: YF-S201           GPIO _
    Water level: HC SR04          GPIO _       
    Water pH: SEN0169V2 pH meter  GPIO _ 

For control:
    4 x peristaltic pumps         GPIO 27 14 12 13
    2 chnl relay and soln valve   GPIO 16 17 18

  - Creating separate functions for data acquisition and nutrient dosing
  - Nutrient dosing algorithm (for remote automation)                         
  - pH monitoring/control functions

# MQTT topics to subscribe
    WATERTEMP = "hydro/water/temp"
    WATERLEVEL = "hydro/water/waterlevel"
    WATERTDS = "hydro/water/tds"
    WATERFLOWRATE = "hydro/water/flowrate"
    WATERNUTRES = "hydro/water/nutres"
    IRRSTATUS = "hydro/water/irrstatus"

# MQTT topics to publish
    IRRIGATION = "hydro/water/irrigation"
    NUTDOS = "hydro/water/nutdos"

"""

MQTT_SERVER = '192.168.1.10'
MQTT_UN = 'hydropo'
MQTT_PW = 'hydropo'
MQTT_TOPIC = 'hydro/+/+'


#The callback for when a PUBLISH message is received
def on_message(client, userdata, msg):
    #t = [float(x) for x in msg.payload.decode("utf-8")]
    print("Message received" + str(msg.payload.decode("utf-8")))
    print("Topic" + str(msg.topic))


#def on_subscribe(client, userdata, mid):
#    print("");


#def on_publish(client, mid, granted_qos):
#    print("");



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



########################################################################################################################################################
###--------------------------------------------------------------- IMPORTS --------------------------------------------------------------------------###
########################################################################################################################################################

import os
import time
import paho.mqtt.client as mqtt

# library imports here

# Libraries for DS18B20 --------------------------------------------------------------
#from w1thermsensor import W1ThermSensor


# Libraries for Grove TDS Meter ------------------------------------------------------
#import math
#import sys
#from grove.adc import ADC

# Libraries for YF-S201 --------------------------------------------------------------

# Libraries for HC SR04 --------------------------------------------------------------

# Libraries for SEN0169V2 pH meter ---------------------------------------------------


# Sensor imports here
import sensors.spectraldata

########################################################################################################################################################
###------------------------------------------------------ GLOBAL DEFINITIONS/VARIABLES --------------------------------------------------------------###
########################################################################################################################################################

# ------- MQTT GLOBALS FOR ESP32 COMMUNICATIONS ------- #

# MQTT topics to subscribe
WATERTEMP = "hydro/water/temp"
WATERLEVEL = "hydro/water/waterlevel"
WATERTDS = "hydro/water/tds"
WATERFLOWRATE = "hydro/water/flowrate"
WATERNUTRES = "hydro/water/nutres"
IRRSTATUS = "hydro/water/irrstatus"

# MQTT topics to publish
IRRIGATION = "hydro/water/irrigation"
NUTDOS = "hydro/water/nutdos"

# ------- MONITORING VARIABLES FOR 'PUBLISH_DATA()' ------- #

# for light data, top panel is lightData_1, 2 middle, 3 bottom
lightData_1 = 0
lightData_2 = 0     
lightData_3 = 0




########################################################################################################################################################
###--------------------------------------------------------------- FUNCTION CALLS -------------------------------------------------------------------###
########################################################################################################################################################

# ------------------------------------------------------------------------------------------------------ #
#                                   General use functions
# Description:  Functions for general usage
#
# Function calls:
#
# ------------------------------------------------------------------------------------------------------ #

# ------------------------------------------------------------------------------------------------------ #
#                               PHASE 1 | SYSTEM INITIALIZATION
# Description:  Starts everytime an automation procedure is chosen as mode of system operation
#
#
# Function calls:
#       establishWIFI_con() -> establish wifi
#       checkWifi_con() -> checks wifi connection/status
#       checkWeb_con() -> establish and checks web connection
#       checkESP32_con() -> establish and checks esp32 connection
#
#       offAllActuators() -> turns off ALL actuators in system (relays, valves, lights)
#
# ------------------------------------------------------------------------------------------------------ #

def establishWIFI_con(): # establish WIFI
    return True

def checkWIFI_con(): # checks WIFI status
    establishWIFI_con()
    return True

def checkWeb_con():
    return True

def checkESP32_con(client, userdata, flags, rc): #The callback for when the client receives a CONNACK response from the server 
    if rc == 0:
        print("Client is connected.")
        client.subscribe(MQTT_TOPIC)
    else:
        print('Client is not connected. Result code is ' + str(rc))

def offAllActuators():
    return True



# ------------------------------------------------------------------------------------------------------ #
#                               PHASE 2 | SYSTEM SETUP
# Description:  Input all necessary parameters for new setup
#               Communicate with web app for input 
#
# Function calls:
#       runSRP() -> runs SYSTEM RESPONSE PROTOCOL, checks if system modules are responding/working
#
#       receiveHostParameters() -> receives necessary input setup parameters coming from webhost/user
#
# 
# ------------------------------------------------------------------------------------------------------ #

def runSRP(): # SYSTEM RESPONSE PROTOCOL, main function call to check if system is working
    """
        SR : Signal Response -> for checking logic signal responses
        1. Check light panels
            -> are the 3 light panels and light sensor working?
                -> do PWM adjusting for
                    Red
                    Blue
                    Green
                    All colors (white)
                -> do again for next panel
                -> if all panels are done, continue
                -> check light switches if working
                    -> ON then OFF pattern for each light module
        2. Check irrigation module
            -> is water level sensor working?
                -> if YES, then skip, ELSE, fill nut res
                -> is water level sensor working? SR must be HIGH
            -> are valves working>
                -> 
    """
    return True

# ------------------------------------------------------------------------------------------------------ #
#                               PHASE 3 | SYSTEM CONFIGURATION
# Description:  Execute all setup preparation
#               A) Prepare nutrient mix for setup
#               B) Prepare light configuration
#               C) Set cultivation time
#
# Function calls:
#       
#       prepareNutrientReservoir() -> fills reservoir with water, mix pH EC and nutrient solutions, balance with water if necessary
# ------------------------------------------------------------------------------------------------------ #

def prepareNutrientReservoir():
    return True



# ------------------------------------------------------------------------------------------------------ #
#                               PHASE 4 | SYSTEM AUTOMATION
# Description:  Automation mode, all of these must run in parallel
#               Will run under 'multiprocessing'
#
# Function calls:
#       monitor_lightquality() -> runs MLR algo and checks if PPFD of setup matches
#       monitor_nutrientreservoir() -> checks if pH, TDS, and EC values are in appropriate levels, check if water level is ok
#       [PHASE 1] checkWifi_con() -> for checking if wifi is still on
#       [PHASE 1] checkESP32_con() -> for checking if client is still connected to the broker/server via MQTT
#
#       PUBLISH_DATA() -> sends live monitoring data, will this also communicate with database?
#
#
#
# ------------------------------------------------------------------------------------------------------ #

def monitor_nutrientreservoir():
    return True

def PUBLISH_DATA():
    return True 