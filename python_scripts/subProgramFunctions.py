import numpy as np
from time import sleep
from scipy import signal

def initial_diagnostics(forceSensor, distanceSensor, window): 
    # for now, just distance sensor and force sensor
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
    sleep(2) # standing by

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

def serial_routine(serial_object): # Interface with LCD GUI controlled by Arduino

    if serial_object.in_waiting > 0: # --> if there is data in buffer
        command = serial_object.readline().decode('utf-8').rstrip()    

    return command # Activation code string to select either the three sub-program

def sensor_interface():
    return 0

def get_force_reading(gravity, force_sensor, window):
    return gravity*force_sensor.get_weight_mean(window)/1000

def diff_eq_coeff(den, freq):
    # calculates the difference equation coefficients based on 
    # spring and damper
    num = 1
    sysModel_TFs = signal.TransferFunction(num, den)
    dt = 1/freq
    sysModel_TFz = sysModel_TFs.to_discrete(dt, method = 'ggbt', alpha = 0.5)
    b_i = sysModel_TFz.num
    a_i = -sysModel_TFz.den

    return a_i, b_i

class admittance_type_haptic:
    # Not sure if the second order force to position system
    # should use OOP

    def __init__(self):
        pass
