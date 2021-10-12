// Based on accelstepper library:
// - http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// Original example written by Jeremy Fielding:
// - https://github.com/jengineer1/Stepper_Motor_Controller
// - https://youtu.be/QMgckRoRy38

// Copyright (c) 2021, Edgar B. Sutawika

#include <AccelStepper.h>

// 1. Defining pins
#define stepperPulse 7
#define stepperDirection 8
#define stepperEnable 9

// 2. Command objects
String stringCommand; // General command from high-level controller (waiting commands)
int activationCode; // activation code for commands '2' and '3'


// 3. Speeds & Limitations
/* List of Max speed for active mode
 *  
 *  2500 pulse/s = 50 mm/s
 *  1000 pulse/s = 20 mm/s
 *  750 pulse/s = 15 mm/s
 *  500 puse/s = 10 mm/s
 */
int activeMaxSpeed = 750;
int passiveMaxSpeed_contCommand = 500; // during continuous command mode
volatile int sliderMaxSpeed; // maximum slider speed depending on parsed parameters

// 4. Declare object for motor actuator
AccelStepper motor_actuator(1, stepperPulse, stepperDirection);

// 5. helpful terms for motor control
long Motor2position = 0;
long temp;
long motor2position2 ;


//===***PID UTILITIES****===//

// 6. Timer for closed-loop knee angle control
void TimerInit(){
  /* Initialize timer1 
   * Sampling Frequency = 100 Hz 
   * Timer for PID loop in passive training (angle control)  
   */ 
  noInterrupts(); //disable all interrupts
  TCCR1A = 0; // Timer or Counter Control Register
  TCCR1B = 0; // The prescaler can be configure in TCCRx
  TCNT1  = 0; // Timer or Counter Register. The actual timer value is stored here.

  OCR1A = 6249;//31249;//6249; //31250; //624;           // Output Compare Match Register (16Mhz/256/10Hz)
  TCCR1B |= (1 << WGM12);  // CTC (Clear Time on Compare Match)
  TCCR1B |= (1 << CS12);   // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // Interrupt Mask Register (Enable Output Compare Interrupt)
  interrupts();            // Enable all interrupts
}

// 7. Global variables and function for PID Control

/*global variables*/
float e2 = 0, e1 = 0, e0, u2, u1, u0; // PID variable computation
float a0, a1, a2, b0, b1, b2;
float ku1, ku2, ke0, ke1, ke2;

float r; // reference command
float y; // plant output

float Kp = 1; // proportional
float Ki = 0; // integral
float Kd = 0; // derivative

float N = 20; // filter coeff
float Ts = 0.1; // 100 Hz sample frequency

/*methods & functions*/
void init_pid(float Kp, float Ki, float Kd, float N, float Ts);
void pid_execute();

//======*******=======//

// 8. Passive training control utilities
const int angleSensorPin = A0; // pot at knee mechanism
volatile int sensorValue = 0;
volatile int measuredAngle = 0; // value read from pot
const int offsetAngle = 44; // systematic offset from absolute potensiometer reading
volatile int targetAngle; // target angle value (for single/continuous command)
volatile int maxMotorSpeed; // deliberate saturation speed for PID
volatile bool pidGo; // Go-NoGo for pid controller
//Note: angle is in Degrees

void setup() {               
  Serial.begin(115200); //Serial.setTimeout(500);

  // Pin setup
  pinMode(stepperPulse, OUTPUT);
  pinMode(stepperDirection, OUTPUT);
  pinMode(stepperEnable, OUTPUT);

  // Pin init
  digitalWrite(stepperPulse, LOW);
  digitalWrite(stepperDirection, HIGH);
  digitalWrite(stepperEnable, LOW);
}

void loop() {
  // Note: all discrete commands are marked by \n string at the end
  
  if (Serial.available()>0){
    stringCommand = Serial.readStringUntil('\n');
    
    //===COMMAND '0': Passive standby - position tracking===
    if (stringCommand.charAt(0) == '0'){
      /*Algorithm
      * 1. extract command code (code format: '0;XXX\n')
      * 2. read the position target value denoted in XXX
      * 3. send XXX value as reference position 
      * 
      * activation code for passive mode:
      * '0;XXX\n' ==> continuous command code;target position
      * '1;XXX;YYY;ZZZ;AAA\n' ==> single command code;max angle;min angle;speed;duration
      */
      bool passiveStandby = true;
      init_pid(Kp, Ki, Kd, N, Ts); 
      TimerInit();
       
      while (passiveStandby) {
        if (Serial.available()>0){
          stringCommand = Serial.readStringUntil('\n');
          
          // if and else if statement to see if string command
          // is either '0' or "-s"
          if (stringCommand.charAt(0) == '0'){
            // a. Update max speed
            maxMotorSpeed = passiveMaxSpeed_contCommand;

            // b. Reference angle 
            stringCommand = Serial.readStringUntil('\n');
            String target_angle_str = getValue(stringCommand, ';', 1);
            targetAngle = target_angle_str.toInt();
  
            // c. Measured angle
            sensorValue = analogRead(angleSensorPin);
            measuredAngle = map(sensorValue, 0, 1023, 0, 333)-offsetAngle; // map it from 0 to 333 degrees)   
            
            //Note: update the reference and measured angle @ t = n for 
            // control loop @ t = n.

          } // end if
          else if (stringCommand == "-s"){
            passiveStandby = false; // Exit passive standby loop
          } // end elif
          
        } // end if serial available
        
      } // end while passive standby loop    
      
    } // end if nest 2
    
    else if (stringCommand.charAt(0) != '0'){
      
      switch (stringCommand.charAt(1)){

        //===COMMAND '1': Passive training Mode===
        case '1':
          /* activation for passive mode:
           *  '1;XXX;YYY;ZZZ;AAA\n' ==> single command code;max angle;min angle;speed;duration
           */
              
          break;
              
        //===COMMAND '2': Assistive Training Mode===
        case '2':
          /* activation for assistive mode is handled by 
           *  high level controller           *  
           */
          bool activeGo = true;
          while (activeGo) {
            if(Serial.available()>0){
              String stopCode = Serial.readStringUntil('\n');
              if (stopCode == "-s"){
                activeGo = true;
              }
            }
          }
          break;
            
        //===COMMAND '3': Full-active training Mode===
        case '3': //it's actually the same as case '2'.
          /* activation for assistive mode is handled by 
           *  high level controller           *  
           */
    
              
          break;
              
        default:
          // do nothing
          break;
      } // end switch case
    } // end elif nest 2
  } // end if nest 1
}// end void loop

/******************************/

// ================================
// Sub-program functions & methods
// ================================

void active_low_level_loop(){
  /* Actuation for active training commands
   *  
   *  Args:
   *    N/A
   */

  if (Serial.available() > 0) {
    String Motor2positionString = Serial.readStringUntil('\n');
    Motor2position = Motor2positionString.toInt();
    
    //Motor2position = Serial.parseInt();
    temp = Motor2position;
    motor2position2 = Motor2position;
  } 
  else {
    motor2position2 = temp;
  }
  
  motor_actuator.moveTo(-motor2position2); // absolute position target
  motor_actuator.run();
}

void passive_low_level_loop(int theta_reference){
  /* Actuation for passive training commands in
   *  single commands
   *  
   *  Args:
   *    theta_reference [int]: Setpoint knee angle.
   */
    
  TimerInit();
}

ISR(TIMER1_COMPA_vect){
  /* Interrupt service routine for
   *  PID execution
   */
  if (pidGo == true){
    pid_execute(targetAngle, measuredAngle, maxMotorSpeed);
  } 
}

void init_active_speed_param(){
  /* Initializing speed parameters for active 
   *  training commands. Active training commands 
   *  are labeled '2' and '3'.
   *  
   *  Args:
   *    N/A
   */
  // 
  motor_actuator.setMaxSpeed (activeMaxSpeed);  
  motor_actuator.setSpeed(500);
  motor_actuator.setAcceleration(1000);
}
 
void init_passive_speed_param(String passive_act_code){
  /* Initializing speed parameters for passive 
   *  training commands. Passive training commands 
   *  are labled '1'.
   *  
   *  Args:
   *    passive_act_code [string]: activation code for passive training
   */
  String slider_speed = getValue(passive_act_code, ';', 3);
  sliderMaxSpeed = slider_speed.toInt();
  motor_actuator.setMaxSpeed(sliderMaxSpeed);
  motor_actuator.setSpeed(0);
  motor_actuator.setAcceleration(800);
}

String getValue(String command_data, char separator, int index){
  /* This code is thanks to the people of stackOverflow <3!!
  */
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = command_data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(command_data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
  }

  return found>index ? command_data.substring(strIndex[0], strIndex[1]) : "";
}

/*==== PID execution routine ====*/
void init_pid(float Kp, float Ki, float Kd, float N, float Ts){
  /* Initializing PID equation based on user defined parameters
   * 
   *  Args:
   *    Kp [float]: proportional gain  
   *    Ki [float]: integral gain
   *    Kd [float]: derivative gain
   *    N [float]: filter coefficient
   *    Ts [float]: sample time in seconds
   */
   
  a0 = (1+N*Ts);
  a1 = -(2+N*Ts);
  a2 = 1;
  b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
  b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
  b2 = Kp + Kd*N;
  ku1 = a1/a0; ku2 = a2/a0; ke0 = b0/a0; ke1 = b1/a0; ke2 = b2/a0;
}

void pid_execute(float target_angle, float plant_output, float speed_sat){
  /* Executing PID control algorithm
   * 
   *  Args:
   *    target_angle [float]: reference angle [deg]
   *    plant_output [float]: measured angle [deg]
   *    speed_sat [float]: max/min speed of motor [pulse/s] (aka speed saturation)
   */

  e2 = e1; 
  e1 = e0; 
  u2 = u1;
  
  r = target_angle;
  y = plant_output;
 
  e0 = r - y; // compute error
  u0 = -ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2; 

  if (u0 > speed_sat){
    u0 = speed_sat;
    motor_actuator.setSpeed(u0);
    motor_actuator.runSpeed();  
  }

  else if (u0 < -speed_sat){
    u0 = -speed_sat;
    motor_actuator.setSpeed(u0);
    motor_actuator.runSpeed();
  }
}
