// Based on accelstepper library:
// - http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// Original example written by Jeremy Fielding:
// - https://github.com/jengineer1/Stepper_Motor_Controller
// - https://youtu.be/QMgckRoRy38

// Copyright (c) 2021, Edgar B. Sutawika

/* ==kill list==
  1. limit switches with two external interrupt
  2. 

*/
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
 *  500 pulse/s = 10 mm/s
 * 
 *  motor speed range for passive mode 100 RPM (666 P/s) - 300 RPM (2000 P/s)
 *  
 *  666 P/s = 100 RPM --> test
 *  800 P/s = 120 RPM --> 10%
 *  933 P/s = 140 RPM --> 20%
 *  1066 P/s = 160 RPM --> 30%
 *  1200 P/s = 180 RPM --> 40%
 *  1333 P/s = 200 RPM --> 50%
 *  1466 P/s = 220 RPM --> 60%
 *  1600 P/s = 240 RPM --> 70%
 *  1733 P/s = 260 RPM --> 80%
 *  1866 P/s = 280 RPM --> 90%
 *  2000 P/s = 300 RPM --> 100%
 * 
 */
int activeMaxSpeed = 750;
int passiveMaxSpeed_contCommand = 500; // during continuous command mode
volatile int sliderMaxSpeed; // maximum slider speed depending on parsed parameters

// 4. Declare object for motor actuator
AccelStepper motor_actuator(1, stepperPulse, stepperDirection);

// 5. helpful terms for motor control
long Motor2position = 0;
long temp;
long motor2position2;
const int interruptPin1 = 2; // front limit switch
const int interruptPin2 = 3; // rear limit switch


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
float Ts = 0.1; // 10 Hz sample frequency

/*methods & functions*/
void init_pid(float Kp, float Ki, float Kd, float N, float Ts);
void pid_execute();

//======*******=======//

// 8. Passive training control utilities
const int angleSensorPin = A0; // pot at knee mechanism
volatile int sensorValue = 0;
volatile int measuredAngle = 0; // value read from pot
const int offsetAngle = 64; // systematic offset from absolute potensiometer reading
volatile int targetAngle; // target angle value (for single/continuous command)
volatile int maxMotorSpeed; // deliberate saturation speed for PID
volatile bool pidGo = false; // Go-NoGo for pid controller
volatile bool activeGo = false;
//Note: angle is in Degrees

/*******====================================*****/
/*******====================================*****/

void setup() {               
  Serial.begin(115200); //Serial.setTimeout(500);

  // Pin setup
  pinMode(stepperPulse, OUTPUT);
  pinMode(stepperDirection, OUTPUT);
  pinMode(stepperEnable, OUTPUT);

  // Pin init
  digitalWrite(stepperPulse, LOW);
  digitalWrite(stepperDirection, HIGH);
  digitalWrite(stepperEnable, HIGH);

  // Limit switch Interrupt
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), front_limit_switch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), rear_limit_switch, CHANGE);

  // Enable PID and internal interrupt
  init_pid(Kp, Ki, Kd, N, Ts); 
  TimerInit();
}

void loop() {
  // Note: all discrete commands are marked by \n string at the end
  
  if (Serial.available()>0){
    stringCommand = Serial.readStringUntil('\n');

    /*===
    From reading stringCommand, there are two main SCENARIOS that would occur
    ===*/

    // ==== SCENARIO 1, if command entered is '0' === //
    if (stringCommand.charAt(0) == '0'){

    //===COMMAND '0': Passive standby - position tracking===
      /*Algorithm
      * 1. extract command code (code format: '0;XXX\n')
      * 2. read the position target value denoted in XXX
      * 3. send XXX value as reference position 
      * 
      * activation code for '0' passive mode:
      * '0;XXX\n' ==> continuous command code;target position
      * 
      */
      bool passiveStandby = true;

      while (passiveStandby) {
        if (Serial.available()>0){
          stringCommand = Serial.readStringUntil('\n');
          
          // if and else if statement to see if string command
          // is either '0' or "-s"
          if (stringCommand.charAt(0) == '0'){
            // a. Update max speed
            pidGo = true;
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
            pidGo = false;
          } // end elif
        } // end if serial available        
      } // end while passive standby loop          
    } // end if nest 2

    // ==== SCENARIO 2, if command entered is NOT'0' === //
    else if (stringCommand.charAt(0) != '0'){
      
      switch (stringCommand.charAt(0)){

        //===COMMAND '1': Passive training Mode===
        case '1':
          /* activation for passive mode:
           *  '1;XXX;YYY;ZZZ;AAA\n' ==> single command code;max angle;min angle;speed;duration
           */
          passive_mode_control(stringCommand); // parse single command
          pidGo = true;

          while(pidGo){

          }
          pidGo = false;
          break;
              
        //===COMMAND '2': Assistive Training Mode===
        case '2': //it's actually the same as case '3' with some modifications.
          /* activation for assistive mode is handled by 
           *  high level controller           *  
           */
          init_active_speed_param();
          activeGo = true;
          while (activeGo) {
            active_mode_control(activeGo);   
          }
          break;
            
        //===COMMAND '3': Full-active training Mode===
        case '3': 
          /* activation for assistive mode is handled by 
           *  high level controller           *  
           */
          init_active_speed_param();
          activeGo = true;
          while (activeGo) {
            active_mode_control(activeGo);   
          }          
          break;
              
        default:
          // do nothing
          break;
      } // end switch case
    } // end elif nest 2
  } // end BIG IF
}// end void loop


/*******====================================*****/
/*******====================================*****/

// ================================
// Sub-program functions & methods
// ================================

// -----------------------
// I. For active mode
//------------------------
void init_active_speed_param(){
  /* Initializing speed parameters for active 
   *  training commands. Active training commands 
   *  are labeled '2' and '3'.
   *  
   *  Args:
   *    N/A
   */
  // 
  motor_actuator.setMaxSpeed(activeMaxSpeed);  
  motor_actuator.setSpeed(1000); //should be the safe number
  motor_actuator.setAcceleration(1000);
}

void active_mode_control(bool activeGo) {
  /* Actuation for active training commands
   *  
   *  Args:
   *    activeGo: status of active mode control
   */

  if (Serial.available() > 0) {
    String Motor2positionString = Serial.readStringUntil('\n');

    if (Motor2positionString == "-s"){ // if stop code is sent, STOP THE PROGRAM
      activeGo = false;
    }
    
    else {
      Motor2position = Motor2positionString.toInt();
      temp = Motor2position;
      motor2position2 = Motor2position;
      motor_actuator.moveTo(-motor2position2); // absolute position target
      motor_actuator.run();
    }
  } 

  else {
    motor2position2 = temp;
    motor_actuator.moveTo(-motor2position2); // absolute position target
    motor_actuator.run();
  }
  // NOTE: absolute position is tracked by HIGH LEVEL CONTROLLER (RASPBERRY PI)
}

// -----------------------
// II. For passive mode
// -----------------------
void passive_mode_control(String activationCode){
  /* Actuation for passive training commands in
   *  single commands. This is essentially an activation command parser
   *
   * Command content:
   *  '1;XXX;YYY;ZZZ;AAA\n'==>single command code;max angle;min angle;speed;duration
   *  Args:
   *    activationCode [str]: contains parameter to do single command executions
   */
  
  String cmd_maxAngle = getValue(activationCode, ';', 1);
  String cmd_minAngle = getValue(activationCode, ';', 2);
  String cmd_speed = getValue(activationCode, ';', 3);
  String cmd_duration = getValue(activationCode, ';', 4);

  // 1. max angle target
  float max_ref = float(cmd_maxAngle.toInt());
 
  // 2. min angle target
  float min_ref = float(cmd_minAngle.toInt());

  // 3. Max speed settings
  maxMotorSpeed = speed_selector(cmd_speed);
  motor_actuator.setMaxSpeed(maxMotorSpeed);
  motor_actuator.setSpeed(0);
  motor_actuator.setAcceleration(800);

  // 4. Training Duration
  int minutes = cmd_duration.toInt(); // [minutes]
  int endtime = minutes*60*1000; // [milliseconds]

}

ISR(TIMER1_COMPA_vect){
  /* Interrupt service routine for
   *  PID execution
   */
  if (pidGo == true){
    pid_execute(targetAngle, measuredAngle, maxMotorSpeed);
  } 
}

// -----------------------------
// III. mischellaneous functions
// -----------------------------
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

int speed_selector (String speed_percent){
  /*
  * 800 P/s  = 120 RPM --> 10%
  * 933 P/s  = 140 RPM --> 20%
  * 1066 P/s = 160 RPM --> 30%
  * 1200 P/s = 180 RPM --> 40%
  * 1333 P/s = 200 RPM --> 50%
  * 1466 P/s = 220 RPM --> 60%
  * 1600 P/s = 240 RPM --> 70%
  * 1733 P/s = 260 RPM --> 80%
  * 1866 P/s = 280 RPM --> 90%
  * 2000 P/s = 300 RPM --> 100%
  *
  */
  int MaxSpeed;
  switch (speed_percent.toInt()){ 
  
    case 10:
      MaxSpeed = 800;
      break;

    case 20:
      MaxSpeed = 933;
      break;

    case 30:
      MaxSpeed = 1066;
      break;

    case 40:
      MaxSpeed = 1200;  
      break;

    case 50:
      MaxSpeed = 1333;
      break;

    case 60:
      MaxSpeed = 1466;
      break;
    
    case 70:
      MaxSpeed = 1600;
      break;
    
    case 80:
      MaxSpeed = 1733;
      break;

    case 90:
      MaxSpeed = 1866;
      break;

    case 100:
      MaxSpeed = 2000;
      break;

    default:
      MaxSpeed = 666;
      break;
  } // end switch case

return MaxSpeed;
}

/*=== Safety functions ===*/
void front_limit_switch(){
  //Hardware safety @software level: forcefully stop the driver when 
  //the slider happens to reach the absolute front end of the rail
  digitalWrite(stepperEnable, LOW);
}

void rear_limit_switch(){
  //Hardware safety @software level: forcefully stop the driver when 
  //the slider happens to reach the absolute rear end of the rail
  digitalWrite(stepperEnable, LOW);
}

// -----------------------
// IV. PID functionalities
// -----------------------

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
  
  ku1 = a1/a0; ku2 = a2/a0; 
  ke0 = b0/a0; ke1 = b1/a0; ke2 = b2/a0;
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
