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
#from gpiozero import DistanceSensor

import serial
import time

import numpy as np
#import pandas as pd
#import matplotlib.pyplot as plt
import argparse


# =================================================================
# ==================1. MAIN SYSTEM FUNCTIONS=======================
# =================================================================

# 1. selection of rehabilitation mode
def run_rehab_program(activationCode, force_sensor):

    if activationCode[0] == '1':
        passive_mode(activationCode)
    elif activationCode[0] == '2':
        semi_active_mode(activationCode, force_sensor)
    elif activationCode[0] == '3':
        full_active_mode(activationCode, force_sensor)     

#----------------------------
# 1. For FULL-PASSIVE program
#----------------------------

def passive_mode(activationCode): 
    return 0

def full_passive_position_control():
    return 0

#----------------------------
# 2. For SEMI-ACTIVE program
#----------------------------
# b. main semi-active mode/semi-assistive sub-program
def semi_active_mode(activationCode, force_sensor):
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
    sysModel = admittance_type(damper_spring, freqSample, force_sensor) # initialize dynamic system model

    start_loop = time.time()
    print("ACTIVATION CODE: ", activationCode)
    print("training mode: Semi-assistive")
    print("Assistive constant: ", assistive_constants(activationCode[1]))
    print("Spring damper constant: ", admittance1_constants(activationCode[2]))
    print("System coef denumerator: ", sysModel.a_i)
    print("System coef numerator: ", sysModel.b_i)
    print(" ")
    print("waiting command")
    print(" ")

    sysModel.set_force_window(weightMeanWindow)    
    
    time_count = 0
    while not stopCondition:
        start_loop = time.time()
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        #time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        pos_setpoint = sysModel.haptic_rendering_1()
        actual_pos = spf.command_actuator(pos_setpoint)
        sysModel.haptic_rendering_2(actual_pos)
        command = spf.serial_routine(ser_command)
        capture_time = time.time()
        if (time_count>under_sample_time):
            print("Input Force: ", round(sysModel.force_in0,2),
                  " N. Target position: ", round(sysModel.pos_now,2), " mm.")
            time_count = 0
            
        if command == "-s":
            stopCondition = True
           
        time_count = time_count + sample_period
        
        time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        
def assistive_constants(assistiveConstCode): 
    '''
    Assistive value constants
        values are still placeholders
    '''
    assist_const = {
        '0': 50, # [ ] unit not decided
        '1': 200,
        '2': 100
    }
    return assist_const.get(assistiveConstCode)

def admittance1_constants(admittanceCode): 
    '''
    Spring, mass, damper constants selection (Three options)
    '''
    damper_spring_pair = {
        '0': den_semi_1, 
        '1': den_semi_2,
        '2': den_semi_3,
    }
    return damper_spring_pair.get(admittanceCode)

#----------------------------
# 3. For FULL-ACTIVE program
#----------------------------
# c. main full-active mode sub-program

def full_active_mode(activationCode, force_sensor):
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
    
    if activationCode[1] == "0":
        isotonic_training(activationCode, force_sensor)
    elif activationCode[1] =="1":
        isometric_training(activationCode, force_sensor)
        

def isotonic_training(activationCode, force_sensor): 
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
    sysModel = admittance_type(damper_spring, freqSample, force_sensor) # initialize dynamic system model
    #    sysModel.set_initial_position(round(distance_sensor.distance*1000, 0))

    
    print("ACTIVATION CODE: ", activationCode)
    print("training mode: full-active")
    print("training mode: isotonic")
    print("Spring damper constant: ", admittance2_constants(activationCode[2]))
    print("System coef denumerator: ", sysModel.a_i)
    print("System coef numerator: ", sysModel.b_i)
    print(" ")
    print("waiting command")
    print(" ")

    sysModel.set_force_window(weightMeanWindow)    
    start_code = time.time()
    
    time_count = 0.0
    
    print("GO")
    start_1 = time.time()
    pos_setpoint = sysModel.haptic_rendering_1()
    print(time.time()-start_1)
    actual_pos = spf.command_actuator(pos_setpoint)
    sysModel.haptic_rendering_2(actual_pos)
    command = spf.serial_routine(ser_command)        
    time.sleep(sample_period - (time.time()-start_1))
    
    start_1 = time.time()
    pos_setpoint = sysModel.haptic_rendering_1()
    print(time.time()-start_1)
    actual_pos = spf.command_actuator(pos_setpoint)
    sysModel.haptic_rendering_2(actual_pos)
    command = spf.serial_routine(ser_command)
    print(time.time()-start_1)
    time.sleep(sample_period - (time.time()-start_1))
    
    start_1 = time.time()
    pos_setpoint = sysModel.haptic_rendering_1()
    print(time.time()-start_1)
    actual_pos = spf.command_actuator(pos_setpoint)
    sysModel.haptic_rendering_2(actual_pos)
    command = spf.serial_routine(ser_command)
    print(time.time()-start_1)
    time.sleep(sample_period - (time.time()-start_1))
    
    start_1 = time.time()
    pos_setpoint = sysModel.haptic_rendering_1()
    print(time.time()-start_1)
    actual_pos = spf.command_actuator(pos_setpoint)
    sysModel.haptic_rendering_2(actual_pos)
    command = spf.serial_routine(ser_command)
    print(time.time()-start_1)
    time.sleep(sample_period - (time.time()-start_1))
    
    '''while not stopCondition:
        start_loop = time.time()
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        #time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        pos_setpoint = sysModel.haptic_rendering_1()
        actual_pos = spf.command_actuator(pos_setpoint)
        sysModel.haptic_rendering_2(actual_pos)
        command = spf.serial_routine(ser_command)
        capture_time = time.time()
        
        if (time_count>under_sample_time):
            print("Input Force: ", round(sysModel.force_in0,2),
                  " N. Target position: ", round(sysModel.pos_now,2),
                  " mm. Time: ", time_count,
                  " s. One loop execution: ", round(time.time()-start_loop, 5),
                  " s. Sample time: ", sample_period, " s.") #time.time()-start_loop, 5
            
            time_count = 0
            
        if command == "-s":
            stopCondition = True
           
        time_count = time_count + sample_period
        time.sleep(sample_period - (time.time()-start_loop))'''
        
def isometric_training(activationCode, force_sensor): # Position Control
    stopCondition = False
    damper_spring = admittance2_constants(activationCode[2]) # assign which damper-spring system
    sysModel = admittance_type(damper_spring, freqSample, force_sensor) # initialize dynamic system model
#     sysModel.set_initial_position(round(distance_sensor.distance*1000, 0))
#     sysModel.set_force_window(weightMeanWindow)
    
    print("ACTIVATION CODE: ", activationCode)
    print("training mode: full-active")
    print("training mode: isometric")
    print("Spring damper constant: ", admittance2_constants(activationCode[2]))
    print("System coef denumerator: ", sysModel.a_i)
    print("System coef numerator: ", sysModel.b_i)
    print(" ")
    print("waiting command")
    print(" ")
    
    sysModel.set_force_window(weightMeanWindow)    
    
    time_count = 0
    while not stopCondition:
        start_loop = time.time()
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        #time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        # this time library attempts to make the system sampling frequency
        # consistent at about "freqSample"
        pos_setpoint = sysModel.haptic_rendering_1()
        actual_pos = spf.command_actuator(pos_setpoint)
        sysModel.haptic_rendering_2(actual_pos)
        command = spf.serial_routine(ser_command)
        capture_time = time.time()
        if (time_count>under_sample_time):
            print("Input Force: ", round(sysModel.force_in0,2), " N. Target position: ", round(sysModel.pos_now,2), " mm.")
            time_count = 0
            
        if command == "-s":
            stopCondition = True
           
        time_count = time_count + sample_period
        
        #print(sample_period - ((time.time()-start_loop)%sample_period))
        print(time.time()-start_loop)
        time.sleep(abs(sample_period - ((time.time()-start_loop)%sample_period)))
        
def admittance2_constants(admittanceCode): 
    # Spring, mass, damper constants selection (Three options)
    damper_spring_pair = {
        '0': den_full_1, 
        '1': den_full_2,
        '2': den_full_3,
    }
    return damper_spring_pair.get(admittanceCode)


        
# =================================================================
# ==============0. CONFIGURING GLOBAL CONSTANTS====================
# =================================================================

# 1. pin assignments GPIO
# - load sensor
#doutPin = 20
#pdSCKPin = 21
weightMeanWindow = 1
pre_SetRatio = 231052/1000 # based on raw data--> 231052, 222489 ~= 1000 gram

# - distance sensor
trigger = 23
echo = 24

# - potensiometer. 
potAngleAnalogIn = 17 # check again

# 2. Configuring sensors
#   a. Force sensor
GPIO.setmode(GPIO.BCM) 
force_sensor = HX711(dout_pin=20, pd_sck_pin=21)
force_sensor.set_scale_ratio(pre_SetRatio)  # set ratio for current channel

#   b. Distance sensor
#distance_sensor = DistanceSensor(trigger, echo)
distance_sensor = 1 # let's just leave this out for the mean time

#   c. knee angle sensor

# 3. Other global variables
deviceLocation = '/dev/ttyACM0' # port in raspi
freqSample = 10.0 #15.0#200.0 # [Hz] system operating frequency 500 Hz rencananya
sample_period = 1/freqSample
ser_command = serial.Serial(deviceLocation, 9600, timeout=1) # initialize serial

under_sample_time = 2.0
'''ser_command.flushInput()
ser_command.flush()
ser_command.flushOutput()
'''
#----------------------------
# A. SEMI-ASSISTIVE ADMITTANCE SYSTEM OPTIONS
#----------------------------
# CAUTION: Transfer function is X[mm]/F[N/mm]!!
# Option 1, 2, 3
den_semi_1 = [1, 0.5] # [N.s/mm, N/mm]
den_semi_2 = [1, 0.2] # [N.s/mm, N/mm]
den_semi_3 = [10, 0.7] # [N.s/mm, N/mm]

#----------------------------
# B. FULL-ACTIVE ADMITTANCE SYSTEM OPTIONS
#----------------------------
# Option 1, 2, 3
den_full_1 = [10, 5] # [N.s/mm, N/mm]
den_full_2 = [50, 5] # [N.s/mm, N/mm]
den_full_3 = [5, 0.8] # [N.s/mm, N/mm]

# trial variables

force_data_so_far = []
position_data_so_far = []


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
        print("====main program====\n ==== Rehab-Bot ====\n")
        print("Step 1. Initiating system diagnostics")
        spf.initial_diagnostics(force_sensor, distance_sensor, weightMeanWindow)

        while True: 
            '''
            Main program loop
                When a rehabilitation program is finished, the loop goes back
                to the top of this loop and loops again within the "standby" loop
            '''
            # ====== STEP 2. SYSTEM SELECTION =======
            print("Step 2. System selection\n")
            time.sleep(1)
            print("Standby mode ....waiting user input")
            standby_mode = True

            while standby_mode: # 
                '''
                standby_mode loop
                    this loop just loops around waiting for serial command'''

                # = This is the part where raspi accepts integer from arduino
                activationCode  = spf.serial_routine(ser_command)
                '''print(activationCode)
                print(isinstance(activationCode, str))
                print(activationCode == "-s")
                print("test")
                time.sleep(0.5)'''
                print('standing by ...')
                print(' ')
                
                # => Run rehabilitation procedure based on 
                #    user input through display.
                
                if len(activationCode) == 3 and (not activationCode =="-s"):
                    # ====== STEP 3. RUN PROGRAM =======
                    run_rehab_program(activationCode, force_sensor)
                    standby_mode = False
                    #plt.figure(1)
                    
                
                # standby 2 seconds
                time.sleep(1)

    except (KeyboardInterrupt, SystemExit):  
        # code that executes before exiting after ctrl+C  
        print ("Bye!\n")
        time.sleep(1)
    
    finally:  
        GPIO.cleanup() # this ensures a clean exit  
        print("shutting program down...")
        time.sleep(1)


