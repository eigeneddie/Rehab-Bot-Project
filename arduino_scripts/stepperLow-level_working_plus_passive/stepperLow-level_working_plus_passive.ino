// Based on accelstepper library:
//(http://www.airspayce.com/mikem/arduino/AccelStepper/index.html) 
// Original example written by Jeremy Fielding:
// https://github.com/jengineer1/Stepper_Motor_Controller
// https://youtu.be/QMgckRoRy38

// Script by Edgar B. Sutawika (2021)

#include <AccelStepper.h>


//Defining pins
#define Stepper2Pulse 7
#define Stepper2Direction 8
#define StepperEnable 9

String stringCode;
int activationCode;

//defining terms

long Motor2position = 0;
long temp;
long motor2position2 ;

/* List of Max speed for active mode
 *  
 *  2500 pulse/s = 50 mm/s
 *  1000 pulse/s = 20 mm/s
 *  750 pulse/s = 15 mm/s
 *  500 puse/s = 10 mm/s
 */

int activeMaxSpeed = 750;
int passiveMaxSpeed = 10000;


AccelStepper step2(1, Stepper2Pulse, Stepper2Direction);


void setup() {               
  Serial.begin(115200);
  //Serial.setTimeout(500);
  step2.setMaxSpeed (activeMaxSpeed);  
  step2.setSpeed(500);

  step2.setAcceleration(10000);
  pinMode(Stepper2Pulse, OUTPUT);
  pinMode(Stepper2Direction, OUTPUT);
  pinMode(StepperEnable, OUTPUT);
  digitalWrite(Stepper2Pulse, LOW);
  digitalWrite(Stepper2Direction, HIGH);
  digitalWrite(StepperEnable, LOW);
}

void loop() {

  if (Serial.available()>0){
    stringCode = Serial.readStringUntil('\n');
    
    switch (stringCode.charAt(0)){
      case '1':
        //
        
        break;
        
      case '2':
        bool stopActive = false;
        while(!stopActive){
          
          
          
          if(Serial.available()>0){
            String stopCode = Serial.readStringUntil('\n');
            if (stopCode == "-s"){
              stopActive = true;
            }
          }
        }

        
        break
      case '3': //it's actually the same as case '2'.
        // stuff
        break;
      default:
        // do nothing
        break;
    }
    
  }
      
  
}

void active_low_level_loop(){

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
 
   step2.moveTo(-motor2position2);
   step2.run();
}

void passive_low_level_loop(){
  
}
