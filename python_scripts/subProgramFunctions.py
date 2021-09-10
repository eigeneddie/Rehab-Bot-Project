import numpy as np
import time
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
    time.sleep(2) # standing by

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
    
    List of local class variables:
    -------------------------------    
    force_data 
    position_data
    
    force_in0 
    force_in1 
    force_in2 
    sensorWindow

    pos_out 
    pos_out1
    pos_out2
    pos_init
    pos_now

    a_i
    b_i
    
    """
    gravity = 9.81 # [m/s/s]

    def __init__(self, admittance_constants, freq):
        # Variable for storing force and position data
        self.force_data = []
        self.position_data = []

        # Force and position tracking variables. 
        # Currently @zero IC 
        self.force_in0 = 0
        self.force_in1 = 0
        self.force_in2 = 0
        self.pos_out = 0
        self.pos_out1 = 0
        self.pos_out2 = 0

        # Kinematic variable
        # Currently @zero IC
        self.pos_now = 0 

    def diff_eq_coeff(self, den, freq):
        '''
        Calculates the COEFFICIENTS for system difference equation
        based on spring and damper provided
        '''
        num = 1
        sysModel_TFs = signal.TransferFunction(num, den)
        dt = 1/freq
        sysModel_TFz = sysModel_TFs.to_discrete(dt, method = 'ggbt', alpha = 0.5)
        
        self.b_i = sysModel_TFz.num
        self.a_i = -sysModel_TFz.den

    def new_force_reading(self, gravity, force_sensor):
        '''
        Read the latest force reading (F[n])
        '''
        self.force_in0 = gravity*force_sensor.get_weight_mean(self.sensorWindow)/1000

    def set_initial_position(self, current_distance):
        '''
        Reading the current distance of slider in the 
        rehabilitation system. This uses an ultrasonic sensor
        and is only called once when a sub-program is run
        '''
        self.pos_init = current_distance

    def set_current_position(self, delta_distance):
        '''
        Calculating current position of the slider.
        This uses the internal encoder/hall sensor on the actuator
        '''
        self.pos_now = self.pos_init + delta_distance

    def set_force_window(self, sensor_window):
        '''
        Setting the number of data reading from load cell
        '''
        self.sensorWindow = sensor_window

    # "Get" functions
    def get_current_force_reading(self):
        return self.force_in0
        