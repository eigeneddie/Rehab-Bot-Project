#include <AccelStepper.h>


//Defining pins
#define Stepper2Pulse 7
#define Stepper2Direction 8
#define StepperEnable 9

//defining terms

long Motor2position = 0;
long temp;
long motor2position2 ;
int speedmin = 0; //pulses per second
int speedmax = 4000;  //pulses per second

AccelStepper step2(1, Stepper2Pulse, Stepper2Direction);


void setup() {               
  Serial.begin(9600);
  //Serial.setTimeout(500);
  step2.setMaxSpeed (10000);  
  step2.setSpeed(5000);
  step2.setAcceleration(10000);
  pinMode(Stepper2Pulse, OUTPUT);
  pinMode(Stepper2Direction, OUTPUT);
  pinMode(StepperEnable, OUTPUT);
  digitalWrite(Stepper2Pulse, LOW);
  digitalWrite(Stepper2Direction, HIGH);
  digitalWrite(StepperEnable, LOW);
}

void loop() {
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
  
   step2.moveTo(motor2position2);
   step2.run();

}
