import numpy as np
import time
from scipy import signal
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
# Complimentary functions for the mainProg script
from datetime import datetime

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
            
            ....
            ....add more
    """
    # Local Constants
    gravity = 9.81
    slider_rail_length = 870 # [mm] v

    def __init__(self, 
                 admittance_const, 
                 sampling_frequency,
                 load_cell ):
        '''
        Init a new instance of admittance environment

            Args:
                -admittance_const [float list]: denumerator of the system transfer function of
                                                spring-damper system.
                -sampling_frequency [float]: sampling frequency of the discrete system.
                -load_cell [N]: force sensor object used to sense muscle strength (force) from patient.
        '''
        # Store force sensing object
        self.force_sensor = load_cell
        
        # Variables to be changed later (not during) __init__:
        self.pos_init_absolute = 0
        self.pos_now = 0
        self.sensorWindow = 1
        
        # Variable for storing force and position data
        self.force_data = []
        self.position_data = []
    
        # Force and position tracking variables. 
        # First order system
        # Currently @zero IC 
        self.force_in0 = 0
        self.force_in1 = 0
        self.pos_target = 0
        self.pos_out1 = 0
        
        '''
        Calculating the COEFFICIENTS for system difference equation
        based on spring and damper provided
        '''
        num = 1
        sysModel_TFs = signal.TransferFunction(num, admittance_const)
        dt = 1/sampling_frequency
        sysModel_TFz = sysModel_TFs.to_discrete(dt, method = 'gbt', alpha = 0.5)
        
        self.b_i = sysModel_TFz.num
        self.a_i = -sysModel_TFz.den

    def haptic_rendering_1(self):
        '''
        Position calculation using difference equation.
            Difference equation format: 
            y[n] = a_1*y[n-1] + b_0*x[n] + b_1*x[n-1]

            in admittance term:
            X[n] = a_1*X[n-1] + b_0*F[n] + b_1*F[n-1]
            where X: position, F: force

            Args:
                NA
                    
            ADMITTANCE-type device algorithm (mass-spring-damper)
            1. read force of the user
            2. calculate target position
            3. send corresponding position to low level controller
                (in other words, send how much 
                delta position the motor must move)
            4. CHANGE virtual environment STATE 

        '''
        
        # Step 1. Read force of user
        self.force_in0 = self.new_force_reading()
        self.force_data.append(self.force_in0)
        
        # Step 2. calculate target position
        position_term = self.a_i[1]*self.pos_out1
        force_term = self.b_i[0]*self.force_in0 + self.b_i[1]*self.force_in1
        self.pos_target = position_term + force_term
        
        return self.pos_target
    
    def haptic_rendering_2(self, delta_dist_actual):
        '''
        ADMITTANCE-type device algorithm (mass-spring-damper)
            1. read force of the user
            2. calculate target position
            3. send corresponding position to low level controller
                (in other words, send how much 
                delta position the motor must move)
            4. CHANGE virtual environment STATE 

            Args:
                NA
        '''
        # Step 3
        # this is done outside the object
        
        # Step 4   
        self.set_current_position(delta_dist_actual)
        self.position_data.append(self.pos_now)
                
        # save temporary state 
        self.force_in1 = self.force_in0
        self.pos_out1 = self.pos_now
        
        # sysModel is now @ t = n (new state)

        
    def new_force_reading(self):
        '''
        Read the latest force reading (F[n])

            Args:  
                NA
            '''
        self.force_in0 = self.gravity*self.force_sensor.get_weight_mean(self.sensorWindow)/1000
        #self.force_data.append(self.force_in0)
        return self.force_in0
    

    def set_initial_position(self, INIT_distance):
        '''
        Reading the current distance of slider in the 
            rehabilitation system. This uses an ultrasonic sensor
            and is only called once when a sub-program is run
    
            Args:
                current_distance [mm]: sensor reading of slider position
                                       from ULTRASONIC sensor from ORIGIN
        '''
        self.pos_init_absolute = INIT_distance

    def set_current_position(self, DELTA_distance):
        '''
        Track current absolute position.
            This uses the internal encoder/hall sensor on the actuator.

            Args: 
                delta_distance [mm]: data from position sensor (encoder/hall)
        '''
        self.pos_now = self.pos_init_absolute + DELTA_distance



    #------------------------------------------------------
    # Application specific functions (assistive training)
    #------------------------------------------------------

    # code here

    #--------------------------
    # Mischellaneous functions
    #--------------------------
    
    def set_force_window(self, sensor_window):
        '''
        Setting the number of data reading from load cell
            Args: 
                sensor_window []: data size to look at sensor reading
            '''
        self.sensorWindow = sensor_window

def command_actuator(target_delta_distance, serial_object):
    '''
    Send command to low-level controller to move the motor at 
        "delta" position. The command is not absolute position, 
        but incremental position.  
        
        Args:    
    '''
    # Momentary placeholder, end program is not like this.
    # Actual implementation will have the low level controller
    # send the "actual delta distance" through serial to the SBC.
    #actual_delta_distance = target_delta_distance
    #actuator_command = serial.Serial(deviceLocation, 115200, timeout=1)
    pulse = int(target_delta_distance*1/8.0*400.0)
    
    serial_object.write((str(pulse)+'\n').encode('utf-8'))
    actual_delta_distance = target_delta_distance
    
    return actual_delta_distance


def initial_diagnostics(forceSensor, distanceSensor, window): 
    # for now, just distance sensor and force sensor
    # Verify sensors are working
    # a. Force sensor    
    print("====1.A. Setting up Load Cell!======")
    err = True
    err_read_bool = True

    # check if load cell reading is successful
    while err or err_read_bool:
        err = forceSensor.zero()
        if err == True:
            print('Tare is unsuccessful. Retrying')#raise ValueError('Tare is unsuccessful. Retrying')
        else:
            err = False
            print('Tare successful!')
        
        reading = forceSensor.get_raw_data_mean()
        if reading:
            #okay
            print('Reading okay!')
            print(' ')
            err_read_bool = False
        else:
            print('invalid data, retrying')
        
        if (err or err_read_bool) == False:
            print("-Load cell NOMINAL\n")
            time.sleep(1)
            
            
    print("Force detected: ",round(forceSensor.get_weight_mean(window)/1000,2), " N")
    print(" ")
    print("Standing by...")
    print(" ")
    time.sleep(1) # standing by
    
    '''
    # b. Distance sensor (HC-SR04 ultrasonic)
    print("====1.B. Setting up Distance Sensor!======")
    dist_okay = False
    
    
    while not dist_okay:
        print("testing distance sensor")
        
        print(distanceSensor.distance)
        check_sensor = isinstance(distanceSensor.distance, float)
        print(check_sensor)
        if check_sensor == True:
            dist_okay = True
            print('Sensor reading', distanceSensor.distance)
            print('-Distance sensor NOMINAL\n')
        print(dist_okay)
    '''
    #=====================================================
    print("Sensors: Nominal")
    print(" ")
    print("==========================")
    print(" ")
    

def serial_routine(serial_object):
    '''
    Interface with LCD GUI controlled by Arduino
        receives command from LCD user interface for which
        rehabilitation mode to run.
    '''
    
    command = ""
    if serial_object.in_waiting > 0: # --> if there is data in buffer
        command = serial_object.readline().decode('utf-8').rstrip()    

    return command # Activation code string to select either the three sub-program

def csv_name_address (activationCode):
    today=datetime.now()
    todaystr = today.strftime("%d%m%Y%_H%M%S")
    filename = activationCode+todaystr+".csv"
    path = r"/home/pi/rehabilitationProject/rehab-bot-project-raspi-local/python_scripts/commplementary/"
    return path, filename