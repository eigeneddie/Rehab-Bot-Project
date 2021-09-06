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
# 
# MIT license, all rights reserved.

import subProgramFunctions as spf 
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

# 2. Configuring sensors
#   a. Force sensor
GPIO.setmode(GPIO.BCM) 
force_sensor = HX711(doutPin, pdSCKPin)
force_sensor.set_scale_ratio(pre_SetRatio)  # set ratio for current channel

#   b. Distance sensor
distance_sensor = DistanceSensor(trigger, echo)

#   c. knee angle sensor

# 3. Other global variables
deviceLocation = '/dev/ttyACM0' # port in raspi
gravity = 9.81 # [m/s/s]

# Current slider position reading (output)
positon_output = [] # [mm] ---> Y[n] signal output

# Current force reading (input)
force_input = [] # [N] ---> X[n] signal input

# Past values of signals
pos_out1 = 0 # [mm] ---> Y[n-1]
pos_out2 = 0 # [mm] ---> Y[n-1]
force_in0 = 0 # [N] ---> X[n]
force_in1 = 0 # [N] ---> X[n-1]
force_in2 = 0 # [N] ---> X[n-2]
 
#=============================
#=============================
# FULL/SEMI ACTIVE CONSTANTS
#=============================
#=============================

'''
-> Difference equation second order format
    y[n] = a_1*y[n-1] + a_2*y[n-2] + b_0*x[n] + b_1*x[n-1] + b_2*x[n-2]

-> matrix format
    [a_0, a_1, a_2]
    [b_0, b_1, b_2]

'''
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


# =================================================================
# =====================MAIN SYSTEM FUNCTIONS=======================
# =================================================================

def main_prog(): 
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


# ======SYSTEM LEVEL FUNCTIONALITY=======


# ======THREE MAIN PROGRAMS========
#
# 1. For FULL-PASSIVE program
#  To be written

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

def systemModel(sysCoefficient, force_input, forcesensor):
    '''
    -> Difference equation second order format
        y[n] = a_1*y[n-1] + a_2*y[n-2] + b_0*x[n] + b_1*x[n-1] + b_2*x[n-2]

    -> matrix format
        [a_0, a_1, a_2]
        [b_0, b_1, b_2]
    '''
    a_i = sysCoefficient[0, :]
    b_i = sysCoefficient[1, :]

    pos_term = a_i[1]*pos_out1 + a_i[2]*pos_out2
    force_term = b_i[0]*force_in0 + b_i[1]*force_in1 + b_i[2]*force_in2
    pos_out = pos_term + force_term
    return pos_out
 
# Running main program 
try:  
    # here you put your main loop or block of code  
    main_prog()
    
except (KeyboardInterrupt, SystemExit):  
    # here you put any code you want to run before the program   
    # exits when you press CTRL+C  
    print ("Bye!\n") # print value of counter  
  
finally:  
    GPIO.cleanup() # this ensures a clean exit  

#=========
# NOTES & USEFUL LINKS
#=========
# synchronization ? 
# race condition
# https://medium.com/mindful-engineering/multithreading-multiprocessing-in-python3-f6314ab5e23f
# https://www.geeksforgeeks.org/multithreading-in-python-set-2-synchronization/
# https://medium.com/velotio-perspectives/an-introduction-to-asynchronous-programming-in-python-af0189a88bbb
#
#  running code correctly with prog involving GPIO http://raspi.tv/2013/rpi-gpio-basics-3-how-to-exit-gpio-programs-cleanly-avoid-warnings-and-protect-your-pi
#  
# I am yet to meet anyone who has not truly worked hard for thousands of hours in order to accomplish something great. 