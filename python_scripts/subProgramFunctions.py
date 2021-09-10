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

class admittance_type_haptic:
    """
    admittance_type_haptic: this handles the sensor reading, calculation,
    and other stuff concerning the admittance
    """
    gravity = 9.81 # [m/s/s]
    force_data = []
    position_data = []

    def __init__(self, admittance_constants, freq, ):
        # System constants
        [self.a_i, self.b_i] = self.diff_eq_coeff(admittance_constants, freq)
        
        # Force and position tracking variables
        self.force_in0 = 0
        self.force_in1 = 0
        self.force_in2 = 0
        self.pos_out = 0
        self.pos_out1 = 0
        self.pos_out2 = 0

        # Kinematic variable
        self.pos_now = 0 

    def new_force_reading(self, gravity, force_sensor, window):
        self.force_in0 = gravity*force_sensor.get_weight_mean(window)/1000
        return self.force_in0
    
    def set_initial_position(self, current_distance):
        self.pos_init = current_distance

    def set_current_position(self, delta_distance):
        self.pos_now = self.pos_init + delta_distance

    def set_force_window(self, sensor_window):
        self.sensorWindow = sensor_window

    def diff_eq_coeff(self, den, freq):
        # calculates the difference equation coefficients based on 
        # spring and damper
        num = 1
        sysModel_TFs = signal.TransferFunction(num, den)
        dt = 1/freq
        sysModel_TFz = sysModel_TFs.to_discrete(dt, method = 'ggbt', alpha = 0.5)
        b_i = sysModel_TFz.num
        a_i = -sysModel_TFz.den
        return a_i, b_i
        