#!/usr/bin/env python3

# Copyright (c) 2021 Edgar B. Sutawika
# 
# Rehabilitation project for lower extremity stroke patients.
#
#
# Project consist of several scripts running concurrently in a multi-threaded fashion
# mainProg.py: Rehab-bot system execution
# livePlotter.py: plotting important states (Leg pushing-pulling force, position of slider, knee angle, etc.)
# saveDatabase.py: sending the data to a cloud to be accessed by essential parties (e.g. doctors)
#
# MIT license, all rights reserved.

import RPi.GPIO as GPIO  # import GPIO
from hx711 import HX711  # import the class HX711
import numpy as np
import matplotlib.pyplot as plt
import serial
import os
from gpiozero import DistanceSensor
import time
from time import sleep
import pandas as pd

# 1. pin assignments GPIO
# - load sensor
doutPin = 20
pdSCKPin = 21
weightMeanWindow = 20
pre_SetRatio = 128 #masih ngasal

# - distance sensor
trigger = 18
echo = 24

# - potensiometer
potAngleAnalogIn = 17 # check again

# Other global variables
deviceLocation = '/dev/ttyACM0'


# 2. Configuring sensors
#   a. Force sensor
GPIO.setmode(GPIO.BCM) 
force_sensor = HX711(doutPin, pdSCKPin)

#   b. Distance sensor
distance_sensor = DistanceSensor(trigger, echo)

#   c. knee angle sensor

#=============================
#=============================
# FULL/SEMI ACTIVE CONSTANTS
#=============================
#=============================

# =================================================================
# =======1. SEMI-ASSISTIVE ADMITTANCE SYSTEM OPTIONS==============
# =================================================================
# mass = 1 kg, damper = 1 Ns/m, spring = 5 N/m
semiActiveConstants1 = np.array([0, 1.9989955, -0.9990005], 
                                [2.49874750e-07, 4.99749501e-07, 2.49874750e-07])

# mass = 1 kg, damper = 1 Ns/m, spring = 6 N/m
semiActiveConstants2 = np.array([0,  1.9989945, -0.9990005], 
                                [2.49874688e-07, 4.99749375e-07, 2.49874688e-07])

# mass = 1 kg, damper = 1 Ns/m, spring = 10 N/m
semiActiveConstants3 = np.array([0,  1.99899051, -0.9990005 ], 
                                [2.49874438e-07, 4.99748877e-07, 2.49874438e-07])

# =================================================================
# =======2. FULL-ACTIVE ADMITTANCE SYSTEM OPTIONS==================
# =================================================================

# mass = 1 kg, damper = 5 Ns/m, spring = 5 N/m
fullActiveConstants1 = np.array([0, 1.99500749, -0.99501248], 
                                [2.49376248e-07, 4.98752495e-07, 2.49376248e-07])

# mass = 1 kg, damper = 1 Ns/m, spring = 5 N/m                               
fullActiveConstants2 = np.array([0, 1.9989955, -0.9990005], 
                                [2.49874750e-07, 4.99749501e-07, 2.49874750e-07])

# mass = 1 kg, damper = 1 Ns/m, spring = 5 N/m                               
fullActiveConstants3 = np.array([0, 1.9989955, -0.9990005], 
                                [2.49874750e-07, 4.99749501e-07, 2.49874750e-07])


#=============================
#=============================


def initial_diagnostics(forceSensor, distanceSensor): # for now, just distance sensor and force sensor
    # Verify sensors are working

    # a. Force sensor
    print("====1. Setting up Load Cell!======")
    err = False
    read_bool = False

    # check if load cell reading is successful
    while not (err and read_bool):
        err = forceSensor.zero()
        if err == False:
            raise ValueError('Tare is unsuccessful. Retrying')
        else:
            print('Tare successful!')
        
        reading = forceSensor.get_raw_data_mean()
        if reading:
            #okay
            read_bool = True
        else:
            print('invalid data, retrying')
        
        if (err and read_bool) == True:
            print("-Load cell NOMINAL\n")     

    forceSensor.set_scale_ratio(pre_SetRatio)  # set ratio for current channel
    print("current weight: " + forceSensor.get_weight_mean(weightMeanWindow) + "grams")
    print(" ")
    print("Standing by...")
    sleep(4) # standing by

    # b. Distance sensor (HC-SR04 ultrasonic)
    print("====2. Setting up Distance Sensor!======")
    dist_okay = False

    while not dist_okay:
        print("testing distance sensor")
        dist_okay = isinstance(distanceSensor.distance, float)

    if dist_okay == True:
        print('-Distance sensor NOMINAL\n')

    #=====================================================
    print("\n")
    print("Sensors: Nominal")


def main(): 
    global force_sensor, distance_sensor

    # ====== STEP 1. INITIATING SYSTEM DIAGNOSTICS =======
    print("====main program====\n ====Rehab-Bot====\n")
    print("Step 1. Initiating system diagnostics")
    initial_diagnostics(force_sensor, distance_sensor)


    while True:
        # ====== STEP 2. SYSTEM SELECTION =======
        print("Step 2. System selection\n")
        sleep(1)
        print("Standby mode 1....waiting user input")
        # this is the part where raspi accepts integer from arduino

        activationCode  = serial_routine()

        
        run_rehab_program(activationCode)


def sensor_interface():
    return 0


def passive_mode(activationCode):
    return 0

def semi_active_mode(activationCode):
    assistConst = activationCode[1]
    admittance1 = activationCode[2]

#    while True:
        #do something
    

def full_active_mode(activationCode):
    activeModeVar = activationCode[1]
    admittance2 = activationCode[2]
    
    # Constructing Admittance haptic system difference equation
    systemCoef = admittance2_constants(admittance2)
    positionTarget = systemCoef[1]+systemCoef[2]
    #strength_training_option(activeModeVar)

    #while True:
        # do something


def get_force_reading():
    return 0

# ======SERIAL FUNCTIONALITIES=======
def serial_routine():
    
    ser = serial.Serial(deviceLocation, 9600, timeout=1)
    ser.flush()

    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()    

    return line 

# ======SELECTING FULL PASSIVE PROGRAM========
# To be written

# ======SELECTING ADMITTANCE PROGRAMS=========
# 1. General function
#----------------------------
def run_rehab_program(activationCode):
    
    switcher = {
        1: passive_mode(activationCode),
        2: semi_active_mode(activationCode),
        3: full_active_mode(activationCode)     
    }
    switcher.get(activationCode[0])

# 2. For SEMI-ACTIVE program
#----------------------------
def assistive_constants(assistiveConstCode): # Assistive value constants
    # values are still placeholders
    switcher = {
        0: 50,
        1: 200,
        2: 100
    }
    switcher.get(assistiveConstCode)

def admittance1_constants(admittanceCode): # Spring, mass, damper constants selection (Three options)
    switcher = {
        0: semiActiveConstants1, 
        1: semiActiveConstants2,
        2: semiActiveConstants3,
    }
    switcher.get(admittanceCode)

# 3. For FULL-ACTIVE program
#----------------------------
def strength_training_option(strength_option): # full-active training selection (position or admittance control)
    switcher = {
        0: isotonic_training(),
        1: isometric_training()
    }
    switcher.get(strength_option)

def admittance2_constants(admittanceCode): # Spring, mass, damper constants selection (Three options)
    switcher = {
        0: fullActiveConstants1, 
        1: fullActiveConstants2,
        2: fullActiveConstants3,
    }
    switcher.get(admittanceCode)

def isotonic_training(): # Admittance Control
    

    return 0

def isometric_training(): # Position Control
    return 0

def systemModel(sysCoefficient, pos_new, pos_old):
    a_i_coef = sysCoefficient[0, :]
    b_i_coef = sysCoefficient[1, :]

    pos_new[]

    
# Run main program
if __name__=="__main__":
    main()

#=========
# NOTES\
#=========
# synchronization ? 
# race condition
# https://medium.com/mindful-engineering/multithreading-multiprocessing-in-python3-f6314ab5e23f
# https://www.geeksforgeeks.org/multithreading-in-python-set-2-synchronization/
# https://medium.com/velotio-perspectives/an-introduction-to-asynchronous-programming-in-python-af0189a88bbb

# I am yet to meet anyone who has not truly worked hard for thousands of hours in order to accomplish something great. 