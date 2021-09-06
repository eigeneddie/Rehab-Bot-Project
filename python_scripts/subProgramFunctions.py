import numpy as np
import serial 

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

def passive_mode(activationCode):
    return 0

def semi_active_mode(activationCode, admittance_Constants):
    assistConst = activationCode[1]
    admittance1 = activationCode[2]
    # Constructing Admittance haptic system difference equation
    systemCoef = admittance_Constants(admittance1)
    positionTarget = systemCoef[1]+systemCoef[2]
#    while True:
        #do something
    
def full_active_mode(activationCode, position_output):
    activeModeVar = activationCode[1]
    admittance2 = activationCode[2]
    
    # Constructing Admittance haptic system difference equation
    systemCoef = admittance2_constants(admittance2)
    pos_out = systemModel(systemCoef, position_output, force_input, forcesensor)
    position_output.append(pos_out)
    #strength_training_option(activeModeVar)

    #while True:
        # do something
