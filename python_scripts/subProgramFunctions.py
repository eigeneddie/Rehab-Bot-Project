import numpy as np
import time
from scipy import signal

class admittance_type:
    """
    admittance_type_haptic: this handles the sensor reading, calculation,
    and other stuff concerning the admittance
    
    List of local class variables:
    -------------------------------    
    force_data: accumulates force data from force sensor (load-cell). 
    position_data: accumulates position data
    
    force_in0: Latest force reading
    force_in1: Second-latest force reading 
    sensorWindow: Data window for reading load-cell

    pos_out: Latest position data
    pos_out1: Second-latest position data
    pos_init: Absolute position at the Beginning of the 
              training program
    pos_now: Current absolute position of the training program

    a_i: Position term coefficient
    b_i: Force term coefficient
    
    """
    gravity = 9.81
    
    def __init__(self, 
                 admittance_const, 
                 sampling_frequency ):
        '''
        Init a new instance of admittance environment

        Args:
            admittance_const [float list]: denumerator of the system transfer function of spring-damper system
            sampling_frequency [float]: sampling frequency of the discrete system 
        '''         
        # Variable for storing force and position data
        self.force_data = [0]
        self.position_data = [self.pos_now]

        # Force and position tracking variables. 
        # First order system
        # Currently @zero IC 
        self.force_in0 = 0
        self.force_in1 = 0
        self.pos_out0 = 0
        self.pos_out1 = 0

        # Kinematic variable
        # Currently @zero IC
        self.pos_now = 0 

        '''
        Calculating the COEFFICIENTS for system difference equation
        based on spring and damper provided
        '''
        num = 1
        sysModel_TFs = signal.TransferFunction(num, admittance_const)
        dt = 1/sampling_frequency
        sysModel_TFz = sysModel_TFs.to_discrete(dt, method = 'ggbt', alpha = 0.5)
        
        self.b_i = sysModel_TFz.num
        self.a_i = -sysModel_TFz.den

    def new_force_reading(self, force_sensor):
        '''
        Read the latest force reading (F[n])
        '''
        self.force_in0 = 9.81*force_sensor.get_weight_mean(self.sensorWindow)/1000
        self.force_data.append(self.force_in0)
        return self.force_in0

    def calculate_position_target(self, force_sensor):
        '''
        Position calculation using difference equation.
        Difference equation format: 
        y[n] = a_1*y[n-1] + b_0*x[n] + b_1*x[n-1] 
        '''
        self.force_in0 = self.new_force_reading(force_sensor)
        position_term = self.a_i[1]*self.pos_out1
        force_term = self.b_i[0]*self.force_in0 + self.b_i[1]*self.force_in1
        self.pos_out0 = position_term + force_term
        self.position_data.append(self.pos_out0)

        self.force_in1 = self.force_in0
        self.pos_out1 = self.pos_out0

    def set_initial_position(self, current_distance):
        '''
        Reading the current distance of slider in the 
        rehabilitation system. This uses an ultrasonic sensor
        and is only called once when a sub-program is run
        '''
        self.pos_init_absolute = current_distance

    def set_current_position(self, delta_distance):
        '''
        Keeping track of the current absolute position
        This uses the internal encoder/hall sensor on the actuator.
        '''
        self.pos_now = self.pos_init_absolute + delta_distance

    def set_force_window(self, sensor_window):
        '''
        Setting the number of data reading from load cell
        '''
        self.sensorWindow = sensor_window
    
    #------------------
    # "Get" functions
    #------------------
    def get_current_force_reading(self):
        return self.force_in0
    
    def get_current_position(self):
        return self.pos_now

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
        