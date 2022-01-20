/* Description
  *
  *   This script is for testing passive capabilities only.
  *   
  *   COMMAND '0'
  *   First part of this code deals with with continuous motion 
  *   tracking that comes from the serial buffer
  * 
  *   COMMAND '1'0;3
  *   Second part of this code deals with giving single commands
  *   that will first parse the command signals into parameters, and 
  *   then execute training program based on the parameters parsed.
  *  
  */

#include <AccelStepper.h>
#include <MedianFilter.h>
MedianFilter filterObject(28, 360);
// 1. Defining pins
#define stepperPulse 7
#define stepperDirection 8
#define stepperEnable 9

 /*  motor speed range for passive mode 100 RPM (666 P/s) - 300 RPM (2000 P/s)
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
 */

String stringCommand;
AccelStepper motor_actuator(1, stepperPulse, stepperDirection);

void TimerInit(){
  /* Initialize timer1 
   * Sampling Frequency = 100 Hz 
   * Timer for PID loop in passive training (angle control)  
   * 
   * Study reference:
   * https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
   * 
   */ 
  noInterrupts(); //disable all interrupts
  TCCR1A = 0; // Timer or Counter Control Register
  TCCR1B = 0; // The prescaler can be configure in TCCRx
  TCNT1  = 0; // Timer or Counter Register. The actual timer value is stored here.

  OCR1A = 15624; // Output Compare Match Register (16Mhz/256/<sampling_freq>Hz)
  //62499 (1 Hz);//31249 (2Hz); //15625 (4Hz) //6249 (10Hz); 
  //624 (100Hz); 

  TCCR1B |= (1 << WGM12);  // CTC (Clear Time on Compare Match)
  TCCR1B |= (1 << CS12);   // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // Interrupt Mask Register (Enable Output Compare Interrupt)
  interrupts();            // Enable all interrupts
}

/*global variables*/

// PID variable computation
float e2 = 0, e1 = 0, e0 =0;
float u2 = 0, u1 = 0, u0 = 0; 
float a0, a1, a2, b0, b1, b2;
float ku1, ku2, ke0, ke1, ke2;

float r; // reference command
float y; // plant output

float Kp = 80; // proportional
float Ki = 0; // integral
float Kd = 0; // derivative

float N = 0; // filter coeff
float Ts = 0.25; // 1 Hz sample frequency

/*methods & functions*/
void init_pid(float Kp, float Ki, float Kd, float N, float Ts);
 

// 8. Passive training control utilities
int maxSpeed_contCommand = 900; // during continuous command mode

// angle sensor utilities
const int angleSensorPin = A0; // pot at knee mechanism
int sensorValue = 0;
const int offsetAngle = 64; // systematic offset from absolute potensiometer reading

// PID input and activation parameters
float targetAngle; // target angle value (for single/continuous command)
float measuredAngle = 0; // value read from pot
bool pidGo = false; // Go-NoGo for pid controller
bool led_state = LOW;

// undersampling utilities
unsigned long startTime_dur;
unsigned long startTime; // start time
unsigned long currentTime; // current time
const unsigned long period = 500; //undersampling data period
unsigned long start_program = millis();

// volatile passive mode parameters:
float max_ref; // maximum angle limit
float min_ref; // minimum angle limit
long maxMotorSpeed; // moving speed of passive exercise
unsigned long endTime; //duration of passive exercise

volatile int assignedSpeed;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Serial.setTimeout(500);

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Pin setup
  pinMode(stepperPulse, OUTPUT);
  pinMode(stepperDirection, OUTPUT);
  pinMode(stepperEnable, OUTPUT);

  // Pin init
  digitalWrite(stepperPulse, LOW);
  digitalWrite(stepperDirection, HIGH);
  digitalWrite(stepperEnable, HIGH);

  digitalWrite(LED_BUILTIN, led_state);
  
  // Enable PID and internal interrupt
  init_pid(Kp, Ki, Kd, N, Ts); 
  TimerInit();
  startTime = millis();
}

void loop() {

  motor_actuator.setSpeed(0); 
  motor_actuator.runSpeed(); // making sure the motor stops when command stops
  

  checkSerial();
  
  sensorValue = analogRead(angleSensorPin);
  measuredAngle = float(map(sensorValue, 0, 1023, 0, 333)-offsetAngle); // map it from 0 to 333 degrees)  
  
  currentTime = millis(); // grab time
  if (currentTime-startTime >= period){
    Serial.print("loop ");
    Serial.print(measuredAngle); Serial.print(" ");
    Serial.print(u0); Serial.print(" ");
    //Serial.print(u1); Serial.print(" ");
    //Serial.print(u2); Serial.print(" ");
    Serial.print(e0); Serial.println(" ");
    //Serial.print(e1); Serial.print(" ");
    //Serial.print(e2); Serial.println(" ");
    startTime = currentTime;
  }

}

void checkSerial(){
  
  if (Serial.available() > 0){
    
    stringCommand = Serial.readStringUntil('\n');
      
    // if and else if statement to see if string command
    // is either '0', '1', or "-s"

    // OPTION '0'
    if (stringCommand.charAt(0) =='9'){
      bool check_angle = true;
      while(check_angle){
        if (Serial.available() > 0){
          stringCommand = Serial.readStringUntil('\n');
    
          if (stringCommand == "-s"){
            check_angle = false;
            break;
          }
        }
        sensorValue = analogRead(angleSensorPin);
        measuredAngle = float(map(sensorValue, 0, 1023, 0, 333)-offsetAngle); // map it from 0 to 333 degrees  
        Serial.println(measuredAngle);
      }
      
    }
    if (stringCommand.charAt(0) == '0'){
      pidGo = true;
      startTime = millis();
      Serial.println("Entering 0 mode");
      delay(2000);
      
      // a. Update max speed  
      maxMotorSpeed = maxSpeed_contCommand;  
      motor_actuator.setMaxSpeed(maxMotorSpeed);
      zero_everything();
      while(pidGo){
        
        // b. Keep checking for Reference angle 
        targetAngle = float(getValue(stringCommand,';',1).toInt());
      
        if (Serial.available() > 0){
          stringCommand = Serial.readStringUntil('\n');

          if (stringCommand == "-s"){
            pidGo = false;
            break;
          }

          else if (stringCommand.charAt(0) == '0'){
            targetAngle = float(getValue(stringCommand,';',1).toInt());
          }
        }

        // c. Measured angle
        sensorValue = analogRead(angleSensorPin);
        measuredAngle = float(map(sensorValue, 0, 1023, 0, 333)-offsetAngle); // map it from 0 to 333 degrees  

        //Note: update the reference and measured angle @ t = n for 
        // control loop @ t = n.
        // e. run motor
        motor_actuator.setSpeed(assignedSpeed);
        motor_actuator.runSpeed(); 

        // d. undersampling serial print
        currentTime = millis();
        if (currentTime-startTime >= period){
          Serial.print("'0' ");
          Serial.print(measuredAngle); Serial.print(" "); //print to raspberry pi system
          Serial.print(u0); Serial.print(" ");
          //Serial.print(u1); Serial.print(" ");
          //Serial.print(u2); Serial.print(" ");
          Serial.print(e0); Serial.println(" ");
          //Serial.print(e1); Serial.print(" ");
          //Serial.print(e2); Serial.println(" ");
          startTime = currentTime;
        }
      }
    } 

    // OPTION '1'
    if(stringCommand.charAt(0) == '1'){

      passive_mode_control(stringCommand); // parsing single command
      pidGo = true;
      bool from_front = true, from_rear = !from_front; // initialize direction to max_ref
      // front: near motor (min_ref domain)
      // rear: far from motor (max_ref domain)
      Serial.println("Entering 1 mode");
      Serial.print("Max angle: "); Serial.println(max_ref);
      Serial.print("Min angle: "); Serial.println(min_ref);
      Serial.print("Max speed: "); Serial.println(maxMotorSpeed);
      Serial.print("Duration: "); Serial.println(endTime);
      delay(2000);
      startTime = millis();
      startTime_dur = millis();
      zero_everything();
      start_program = millis();

      while(pidGo){

        // a. *Update max speed

        //  *already handled in 'passive_mode_control' (utilizing 'speed' variable)

        // b. Reference angle (utlizing 'max angle' and 'min angle' variable)
        
        // -> assigning target while still on going
        if (from_front == true && from_rear == false){
          if(measuredAngle == max_ref){// -> change direction at the end of the rail
            delay(1000);
            Serial.print("current and target angle: ");Serial.print(measuredAngle); Serial.print(" "); Serial.println(max_ref);
            from_front = !from_front;
            from_rear = !from_rear;  
            targetAngle = min_ref;
            
          }
          else{
            targetAngle = max_ref;
          }
        }
        if (from_front == false && from_rear == true){
          if(measuredAngle == min_ref){
            delay(1000);
            Serial.print("current and target angle: ");Serial.print(measuredAngle); Serial.print(" "); Serial.println(max_ref);
            from_front = !from_front;
            from_rear = !from_rear;
            targetAngle = max_ref;           
          }
          else{
            targetAngle = min_ref;
          }
        }

        // c. Measured angle
        /*sensorValue = analogRead(angleSensorPin);
        measuredAngle = float(map(sensorValue, 0, 1023, 0, 333)-offsetAngle);
        */
        // d. run motor
        motor_actuator.setSpeed(assignedSpeed);
        motor_actuator.runSpeed();
        
        // e. Stoping criteria (utilizing 'duration' variable and force stop "-s")
        if (Serial.available() > 0){
          stringCommand = Serial.readStringUntil('\n');

          if (stringCommand == "-s"){
            pidGo = false;
            break;
          }
        }
        
        // f. undersampling print
        currentTime = millis();
        if (currentTime-startTime >= period){
          Serial.print("'1' ");
          Serial.print(measuredAngle); Serial.print(" "); //print to raspberry pi system
          Serial.print(u0); Serial.print(" ");
          Serial.print(e0); Serial.print(" ");
          Serial.print(targetAngle); Serial.print(" ");
          Serial.println(currentTime-start_program);
          startTime = currentTime;
        }
        
        currentTime = millis();
        if(currentTime-startTime_dur>=endTime){
          pidGo = false;
          break;
        }
      }
    }
  } 
}


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
  max_ref = float(cmd_maxAngle.toInt());
 
  // 2. min angle target
  min_ref = float(cmd_minAngle.toInt());

  // 3. Max speed settings
  maxMotorSpeed = speed_selector(cmd_speed);
  motor_actuator.setMaxSpeed(maxMotorSpeed);
  motor_actuator.setSpeed(0);
  motor_actuator.runSpeed();
  motor_actuator.setAcceleration(800);

  // 4. Training Duration
  unsigned long minutes_sys = (unsigned long)cmd_duration.toInt(); // [minutes]
  endTime = minutes_sys*(unsigned long)60*(unsigned long)1000; // [milliseconds]

}

/*==== PID execution routine ====
 *===============================*/
ISR(TIMER1_COMPA_vect){
  /* Interrupt service routine for
   *  PID execution
   */
  if (pidGo == true){
    sensorValue = analogRead(angleSensorPin);
    measuredAngle = float(map(sensorValue, 0, 1023, 0, 333)-offsetAngle);
    assignedSpeed = int(pid_execute(targetAngle, measuredAngle, maxMotorSpeed));
  } 
  else if (pidGo == false){
    assignedSpeed = 0;
    u0 = 0; u1 = 0; u2 = 0;
    e0= 0; e1 = 0; e2 = 0;
  }
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state; // to check if the timer works
}

void zero_everything(){
  e2 = 0, e1 = 0, e0 =0;
  u2 = 0, u1 = 0, u0 = 0; 
  assignedSpeed = 0;
}
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
  
  ku1 = a1/a0; 
  ku2 = a2/a0; 

  ke0 = b0/a0; 
  ke1 = b1/a0; 
  ke2 = b2/a0;
}

float pid_execute(float target_angle, float plant_output, long speed_sat){
  /* Executing PID control algorithm
   * 
   *  Args:
   *    target_angle [float]: reference angle [deg]
   *    plant_output [float]: measured angle [deg]
   *    speed_sat [float]: max/min speed of motor [pulse/s] (aka speed saturation)
   */

  //e2 = e1; 
  e1 = e0; 
  //u2 = u1;
  u1 = u0;

  r = target_angle;
  y = plant_output;

  float speed_sat_fl = float(speed_sat);
  e0 = r-y; // compute error
  u0 = ke0*e0;// + ke1*e1 + ke2*e2 - ku1*u1; - ku2*u2; //Kp*e0;//-ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2; 
  //u0 = (Kp+Ki*Ts)*e0 - Kp*e1 + u1;
  
  if (u0 >= speed_sat_fl){
    u0 = speed_sat;
  }

  else if (u0 <= -speed_sat_fl){
    u0 = -speed_sat;
  }
  return u0;
}
/*===============================
 *===============================*/

 
//======USEFULL CODES======//

String getValue(String command_data, char separator, int index){
 /* This code is thanks to the people of stackOverflow <3!!
  * convinient for extracting command usage.
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

long speed_selector (String speed_percent){
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
  long MaxSpeed;
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

/*int analogRead(int pin)
{
int total=0;
for(int n=0;n<32;n++)
  total= total + analogRead(pin);
return total/32;
}*/
