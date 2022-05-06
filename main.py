/**
For Thesis Completion ...
Author: Andre Bamba
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
**/