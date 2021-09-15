# Copyright (c) 2021 Edgar B. Sutawika
# 
# Rehabilitation project for lower extremity stroke patients.
#
# Project consist of several scripts running concurrently in a multi-threaded fashion
# All programs are run by raspberry pi 4
# mainProg.py: Rehab-bot system execution. 
# livePlotter.py: plotting important states (Leg pushing-pulling force, position of slider, knee angle, etc.)
# saveDatabase.py: sending the data to a cloud to be accessed by essential parties (e.g. doctors)
# 
# MIT license, all rights reserved.

#!/usr/bin/env python3

import subProgramFunctions as spf 
from subProgramFunctions import admittance_type
import RPi.GPIO as GPIO  # import GPIO
from hx711 import HX711  # import the class HX711
from gpiozero import DistanceSensor

import serial
import time

import numpy as np
import pandas as pd

# =================================================================
# ==============0. CONFIGURING GLOBAL CONSTANTS====================
# =================================================================

# 1. pin assignments GPIO
# - load sensor
doutPin = 20
pdSCKPin = 21
weightMeanWindow = 20
pre_SetRatio = 128 #masih ngasal

# - distance sensor
trigger = 18
echo = 24

# - potensiometer. 
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
freqSample = 200 # [Hz] system operating frequency 500 Hz rencananya
sample_period = 1/freqSample
ser_command = serial.Serial(deviceLocation, 9600, timeout=1) # initialize serial
ser_command.flush()

# =================================================================
# ===============1. FULL/SEMI ACTIVE CONSTANTS=====================
# =================================================================
'''
-> Difference equation first and second order format
    y[n] = a_1*y[n-1] + b_0*x[n] + b_1*x[n-1] 
    y[n] = a_1*y[n-1] + a_2*y[n-2] + b_0*x[n] + b_1*x[n-1] + b_2*x[n-2]

-> matrix format
    [a_0, a_1, a_2, ..., a_n]
    [b_0, b_1, b_2, ..., b_n]

-> update: since the velocity of the system is very slow, inertia effects
    is hugely negligible. We could reduce the system model into first order
    admittance system.
''' 
#----------------------------
# A. SEMI-ASSISTIVE ADMITTANCE SYSTEM OPTIONS
#----------------------------
# CAUTION: Transfer function is X[mm]/F[N/mm]!!
# Option 1, 2, 3
den_semi_1 = [10, 0.5] # [N.s/mm, N/mm]
den_semi_2 = [1, 0.2] # [N.s/mm, N/mm]
den_semi_3 = [10, 0.5] # [N.s/mm, N/mm]

#----------------------------
# B. FULL-ACTIVE ADMITTANCE SYSTEM OPTIONS
#----------------------------
# Option 1, 2, 3
den_full_1 = [10, 0.5] # [N.s/mm, N/mm]
den_full_2 = [1, 0.2] # [N.s/mm, N/mm]
den_full_3 = [10, 0.5] # [N.s/mm, N/mm]

# =================================================================
# ==================2. MAIN SYSTEM FUNCTIONS=======================
# =================================================================

# 1. selection of rehabilitation mode
def run_rehab_program(activationCode):
    prog_option = {
        1: passive_mode(activationCode),
        2: semi_active_mode(activationCode),
        3: full_active_mode(activationCode)     
    }
    prog_option.get(activationCode[0])

# =================================================================
# ==================3. THREE MAIN PROGRAMS=========================
# =================================================================
#----------------------------
# 1. For FULL-PASSIVE program
#----------------------------
#  To be written
# a. main passive mode sub-program 
def passive_mode(activationCode): 
    return 0

def full_passive_position_control():
    return 0

#----------------------------
# 2. For SEMI-ACTIVE program
#----------------------------
# b. main semi-active mode/semi-assistive sub-program
def semi_active_mode(activationCode):
    ''' 
    Semi active-strength exercise
        Constructing Admittance haptic system difference equation for
        ASSISTED resistance training.

        Args:
            activationCode [str]: activation string to determine which damper_spring 
                                  system to be used in the admittance system.
    '''
    stopCondition = False
    assist_level = assistive_constants(activationCode[1]) # assign assistive level of machine
    damper_spring = admittance1_constants(activationCode[2]) # assign which damper-spring system
    sysModel = admittance_type(damper_spring, freqSample) # initialize dynamic system model
    sysModel.set_initial_position(round(distance_sensor.distance*1000, 0))
    sysModel.set_force_window(weightMeanWindow)

    while not stopCondition:
        start_loop = time.time()
        sysModel_n = spf.haptic_rendering(sysModel, force_sensor) # Output system @ latest state
        spf.command_actuator(sysModel_n) # send command to actuator
        
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        command = spf.serial_routine(ser_command)
        time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        if command == "-s":
            stopCondition = True

def assistive_constants(assistiveConstCode): 
    '''
    Assistive value constants
        values are still placeholders
    '''
    assist_const = {
        0: 50, # [ ] unit not decided
        1: 200,
        2: 100
    }
    assist_const.get(assistiveConstCode)

def admittance1_constants(admittanceCode): 
    '''
    Spring, mass, damper constants selection (Three options)
    '''
    damper_spring_pair = {
        0: den_semi_1, 
        1: den_semi_2,
        2: den_semi_3,
    }
    damper_spring_pair.get(admittanceCode)

#----------------------------
# 3. For FULL-ACTIVE program
#----------------------------
# c. main full-active mode sub-program

def full_active_mode(activationCode, admittance_const):
    '''
    Sub-program 3: Patient full-active treatment.
        Patient's strength level has increased into levels
        where they could start actively train their muscle strength.

        full active mode acts like an extremely light "work-out in a gym" 
        training for the stroke patient.

        The resistance of this rehabilitation training strategy uses
        haptic rendering of an admittance environment (inputs force, 
        outputs position). Other resistance strategies could be used such 
        as a friction model.  
        
        ADMITTANCE-type device algorithm (mass-spring-damper)
        1. read force of the user
        2. calculate the resulting position
        3. send corresponding position to low level controller
        4. CHANGE virtual environment STATE

        For analogy with IMPEDANCE-type algorithm (see stanford hapkit)
        1. read position of user (compute position in counts --> to meters)
        2. calculate the resulting force
        3. send corresponding force to motor
        4. change virtual environment of state
    '''
    #strength_option, serial_object, admittanceCode, 
    #                         force_input, position_output, force_sensor): 
    # full-active training selection (position or admittance control)
    strength_training_option = {
        0: isotonic_training(activationCode),
        1: isometric_training(activationCode)
    }
    strength_training_option.get(activationCode[1])

def isotonic_training(activationCode): 
    ''' 
    Full active-strength exercise-isotonic training
        Constructing Admittance haptic system difference equation for
        resistance training.

        Args:
            activationCode [str]: activation string to determine which damper_spring 
                                  system to be used in the admittance system.
    '''
    stopCondition = False
    damper_spring = admittance2_constants(activationCode[2]) # assign which damper-spring system
    sysModel = admittance_type(damper_spring, freqSample) # initialize dynamic system model
    sysModel.set_initial_position(round(distance_sensor.distance*1000, 0))
    sysModel.set_force_window(weightMeanWindow)

    while not stopCondition:
        start_loop = time.time()
        sysModel_n = sysModel.haptic_rendering(sysModel, force_sensor) # Output system @ latest state
        spf.command_actuator(sysModel_n) # send command to actuator
        
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        command = spf.serial_routine(ser_command)
    
        if command == "-s":
            stopCondition = True

def isometric_training(activationCode): # Position Control
    return 0

def admittance2_constants(admittanceCode): 
    # Spring, mass, damper constants selection (Three options)
    damper_spring_pair = {
        0: den_full_1, 
        1: den_full_2,
        2: den_full_3,
    }
    damper_spring_pair.get(admittanceCode)
 
# =================================================================
# ====================4. RUNNING MAIN PROGRAM =====================
# =================================================================

# Running main program 
if __name__=="__main__":
    try:  
        # main loop of program 
        # main_prog()
        # ====== STEP 1. INITIATING SYSTEM DIAGNOSTICS =======
        # run once
        print("====main program====\n ====Rehab-Bot====\n")
        print("Step 1. Initiating system diagnostics")
        spf.initial_diagnostics(force_sensor, distance_sensor)

        while True: 
            '''
            Main program loop
                When a rehabilitation program is finished, the loop goes back
                to the top of this loop and loops again within the "standby" loop
            '''
            # ====== STEP 2. SYSTEM SELECTION =======
            print("Step 2. System selection\n")
            time.sleep(2)
            print("Standby mode 1....waiting user input")
            standby_mode = True

            while standby_mode: # 
                '''
                standby_mode loop
                    this loop just loops around waiting for serial command'''

                # = This is the part where raspi accepts integer from arduino
                activationCode  = spf.serial_routine(ser_command)

                # => Run rehabilitation procedure based on 
                #    user input through display.
                if isinstance(activationCode, str) == True and (not activationCode =="-s"):
                    # ====== STEP 3. RUN PROGRAM =======
                    run_rehab_program(activationCode)
                    standby_mode = False

    except (KeyboardInterrupt, SystemExit):  
        # code that executes before exiting after ctrl+C  
        print ("Bye!\n")
        time.sleep(1)
    
    finally:  
        GPIO.cleanup() # this ensures a clean exit  
        print("shutting program down...")
        time.sleep(2)

#=========
# NOTES & USEFUL LINKS
#=========
# synchronization ? 
# race condition
# https://medium.com/mindful-engineering/multithreading-multiprocessing-in-python3-f6314ab5e23f
# https://www.geeksforgeeks.org/multithreading-in-python-set-2-synchronization/
# https://medium.com/velotio-perspectives/an-introduction-to-asynchronous-programming-in-python-af0189a88bbb
#
# running code correctly with prog involving GPIO: http://raspi.tv/2013/rpi-gpio-basics-3-how-to-exit-gpio-programs-cleanly-avoid-warnings-and-protect-your-pi
# alternatives to if and elif statements: https://medium.com/swlh/3-alternatives-to-if-statements-to-make-your-python-code-more-readable-91a9991fb353
# interrupt program by serial input, maybe: https://stackoverflow.com/questions/50566559/pythons-pyserial-with-interrupt-mode
# serial com raspi-arduino: https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/ 
# I am yet to meet anyone who has not truly worked hard for thousands of hours in order to accomplish something great.
# 
