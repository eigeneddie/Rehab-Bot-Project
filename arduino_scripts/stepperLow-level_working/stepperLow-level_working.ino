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

//defining terms

long position_long = 0;
long temp;
long target_position;

/* List of Max speed for active mode
 *  
 *  2500 pulse/s = 50 mm/s
 *  1000 pulse/s = 20 mm/s
 *  750 pulse/s = 15 mm/s
 *  500 puse/s = 10 mm/s
 */

int activeMaxSpeed = 1500;
int passiveMaxSpeed = 10000;


AccelStepper step2(1, Stepper2Pulse, Stepper2Direction);


void setup() {               
  Serial.begin(115200);
  //Serial.setTimeout(500);
  step2.setMaxSpeed (activeMaxSpeed);  
  step2.setSpeed(500);

  step2.setAcceleration(7000);
  pinMode(Stepper2Pulse, OUTPUT);
  pinMode(Stepper2Direction, OUTPUT);
  pinMode(StepperEnable, OUTPUT);
  digitalWrite(Stepper2Pulse, LOW);
  digitalWrite(Stepper2Direction, LOW);
  digitalWrite(StepperEnable, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    String positionString = Serial.readStringUntil('\n');
    position_long = positionString.toInt();
    
    //Motor2position = Serial.parseInt();
    temp = position_long;
    target_position = position_long;
  }
  else {
    target_position = temp;
  }
  
   step2.moveTo(target_position);
   step2.run();

}
