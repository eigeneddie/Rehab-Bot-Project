import numpy as np
import serial 
import time 

def initial_diagnostics(forceSensor, distanceSensor, window): # for now, just distance sensor and force sensor
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

    print("current weight: " + forceSensor.get_weight_mean(window) + "grams")
    print(" ")
    print("Standing by...")
    time.sleep(3) # standing by

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

def serial_routine(deviceLocation): # Interface with LCD GUI controlled by Arduino
    
    ser = serial.Serial(deviceLocation, 9600, timeout=1)
    ser.flush()

    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()    

    return line # Activation code string to select either the three sub-program

def sensor_interface():
    return 0

def get_force_reading(gravity, force_sensor, window):
    return gravity*force_sensor.get_weight_mean(window)/1000

def isotonic_training(): # Admittance Control

    return 0

def isometric_training(): # Position Control
    return 0
