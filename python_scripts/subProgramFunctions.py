import numpy as np
import time
from scipy import signal

# Complimentary functions for the mainProg script

class admittance_type:
    """
    admittance_type_haptic: this class stores the property of the admittance
    environment system dynamics and its accompanying routines such as calculating 
    the target output state, store states in time, sensor reading, etc. 
        ===============================
        List of local class variables:
        ===============================   
            force_data [N]: accumulates force data from force sensor (load cell). 
            position_data [mm]: accumulates position data
            force_in0 [N]: Latest force reading
            force_in1 [N]: Second-latest force reading 
            sensorWindow [#data]: Data window for reading load-cell
            --------------------------------------------------------
            pos_out [mm]: Latest position data
            pos_out1 [mm]: Second-latest position data
            pos_init [mm]: Absolute position at the Beginning of the 
                           training program (due to the circumstance at 
                           the time of working on this project, I am 
                           unable to use an absolute encoder. An absolute encoder
                           might omit the use of this part of the class)
            pos_now [mm]: Current absolute position of the training program
            ---------------------------------------------------------
            a_i []: Position term coefficient
            b_i [mm/N]: Force term coefficient   
    """
    # Local Constants
    gravity = 9.81
    slider_rail_length = 800 # [mm] v

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

        self.sensorWindow = 20

    def new_force_reading(self, force_sensor):
        '''
        Read the latest force reading (F[n])

            Args:  
                force_sensor [N]: force sensor object using HX711 library
            '''
        self.force_in0 = self.gravity*force_sensor.get_weight_mean(self.sensorWindow)/1000
        self.force_data.append(self.force_in0)
        return self.force_in0

    def calculate_position_target(self, force_sensor):
        '''
        Position calculation using difference equation.
            Difference equation format: 
            y[n] = a_1*y[n-1] + b_0*x[n] + b_1*x[n-1]

            Args:
                force_sensor [N]: sensor reading from HX711 that   
                                  processes the load-cell 
        '''
        self.force_in0 = self.new_force_reading(force_sensor)
        self.force_data.append(self.force_in0)
        position_term = self.a_i[1]*self.pos_out1
        force_term = self.b_i[0]*self.force_in0 + self.b_i[1]*self.force_in1
        self.pos_out0 = position_term + force_term
        self.position_data.append(self.pos_out0) # FLAG

        self.force_in1 = self.force_in0
        self.pos_out1 = self.pos_out0

    def set_initial_position(self, distance_sensor):
        '''Reading the current distance of slider in the 
            rehabilitation system. This uses an ultrasonic sensor
            and is only called once when a sub-program is run
    
            Args:
                current_distance [mm]: sensor reading of slider position
                                       from ULTRASONIC sensor
        '''
        self.pos_init_absolute = distance_sensor

    def set_current_position(self, delta_distance):
        '''
        Track current absolute position.
            This uses the internal encoder/hall sensor on the actuator.

            Args: 
                delta_distance [mm]: data from position sensor (encoder/hall)
        '''
        self.pos_now = self.pos_init_absolute + delta_distance

    def set_force_window(self, sensor_window):
        '''
        Setting the number of data reading from load cell
            Args: 
                sensor_window []: data size to look at sensor reading
            '''
        self.sensorWindow = sensor_window
    
    #------------------------------------------------------
    # Application specific functions (assistive training)
    #------------------------------------------------------

    # code here

    #------------------
    # "Get" functions
    #------------------
    def get_current_force_reading(self):
        return self.force_in0
    
    def get_current_position(self):
        return self.pos_now

def control_loop(sysModel, sensor_input):
    '''
    ADMITTANCE-type device algorithm (mass-spring-damper)
        1. read force of the user
        2. calculate the resulting position
        3. send corresponding position to low level controller
            (iow, send how much delta position the motor must move)
        4. CHANGE virtual environment STATE    
    '''
    # Step 1 & 2
    sysModel.calculate_position_target(sensor_input)

    # Step 3 & 4
    sysModel.set_current_position(sysModel.pos_out0)
    sysModel_n = sysModel
    return sysModel_n

def command_actuator(system_admittance):
    '''
    Send command to low-level controller to move the motor at 
    "delta" position. The command is not absolute position, 
    but incremental position.  
        Args: 
        
    '''
    
    return 0


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
        