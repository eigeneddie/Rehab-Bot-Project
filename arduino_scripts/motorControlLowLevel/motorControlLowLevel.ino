/* Rehabilitation Device for lower extremity: low-level control module
 *  Edgar Sutawika
 *  
 *  Brushless DC motor controller based on 
 *  simpleFOC library: https://docs.simplefoc.com/code
 *  
 *  Modules: 
 *  1. Arduino mega
 *  2. Inineon BLDCIFX007T driver
 *  
 */


/***Part 1: Initiating Motor Control Setup***/
// source: https://docs.simplefoc.com/code

/* Motor low level control */

#include <SimpleFOC.h>
// Sensor variables
#define hallPinA 18
#define hallPinB 19
#define hallPinC 20
#define polePair 7 // 14 poles, 7 pairs

// Driver variables
#define pwmPinA 11//6//11 
#define pwmPinB 10//5//10
#define pwmPinC 9//3//9

#define inhibitA 6
#define inhibitB 5
#define inhibitC 3

//#define enableP 8

// step 1. Position sensor setup (hall sensor)
/*HallSensor(int hallA, int hallB , int cpr, int index)
  - hallA, hallB, hallC    - HallSensor A, B and C pins
  - pp                     - pole pairs
*/
HallSensor sensor = HallSensor(hallPinA, hallPinB, hallPinC, polePair); 
// interrupt routine initialization
void doA() {sensor.handleA();}
void doB() {sensor.handleB();}
void doC() {sensor.handleC();}

// step 2. Driver setup

BLDCDriver3PWM driver = BLDCDriver3PWM(pwmPinA,pwmPinB,pwmPinC, inhibitA, inhibitB, inhibitC); //Coba cek pinnya lagi
//driver.pwm_frequency = 32000;


// step 3. Current sense setup
// Kalo ada, ya.
// IFX007T shield ADA CURRENT SENSE. HARUS DICEK LAGI
// Example: InlineCurrentSense(shunt_resistance, gain, adc_a, adc_b)
//InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50, A0, A2);

// step 4. Motor setup
// BLDCMotor(int pole_pairs)
BLDCMotor motor1 = BLDCMotor(polePair);

//void TimerInit(){
//  //Initialize timer1 
//  //Sampling Frequency = 100 Hz
//  
//  noInterrupts(); //disable all interrupts
//  TCCR1A = 0; // Timer or Counter Control Register
//  TCCR1B = 0; // The prescaler can be configure in TCCRx
//  TCNT1  = 0; // Timer or Counter Register. The actual timer value is stored here.
//
//  OCR1A = 624;           // Output Compare Match Register (16Mhz/256/100Hz)
//  TCCR1B |= (1 << WGM12);  // CTC (Clear Time on Compare Match)
//  TCCR1B |= (1 << CS12);   // 256 prescaler
//  TIMSK1 |= (1 << OCIE1A); // Interrupt Mask Register (Enable Output Compare Interrupt)
//  interrupts();            // Enable all interrupts
//}

void setup() {
  // put your setup code here, to run once:
//  pinMode(inhibitA, OUTPUT);
//  pinMode(inhibitB, OUTPUT);
//  pinMode(inhibitC, OUTPUT);
//
//  digitalWrite(inhibitA, HIGH);
//  digitalWrite(inhibitB, HIGH);
//  digitalWrite(inhibitC, HIGH);
  
  /*MOTOR SETUP AND DRIVER SETUP*/
  //Serial.begin(115200); // for debugging purposes
  // initialize sensor hardware
  sensor.pullup = Pullup::USE_INTERN;
  sensor.enableInterrupts(doA, doB, doC);

  //buat coba
  //Serial.println("sensor ready");
  //_delay(1000);
  // power supply voltage
  driver.voltage_power_supply = 12; // volts
  driver.voltage_limit = 10; // volts

  motor1.linkSensor(&sensor); //position sensor for FOC
  motor1.linkDriver(&driver);
  // motor1.linkCurrentSense(&current_sese); 
 
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.voltage_sensor_align = 3; //default 3V --> motor and sensor alignment
  motor1.sensor_offset = 0; //default 0 rad
  motor1.phase_resistance = 0.02; // Ohms

  
  //====Position control parameters======//
  motor1.controller = MotionControlType::angle; //maybe
   
  motor1.PID_velocity.P = 0.2/motor1.phase_resistance;
  motor1.PID_velocity.I = 20/motor1.phase_resistance;
  motor1.PID_velocity.D = 0.001/motor1.phase_resistance;
  // jerk control it is in Volts/s or Amps/s
  // for most of applications no change is needed 
  motor1.PID_velocity.output_ramp = 1e6;
  
  // velocity low pass filtering time constant
  motor1.LPF_velocity.Tf = 0.01;
  
  // angle loop controller
  motor1.P_angle.P = 20/motor1.phase_resistance;
  motor1.P_angle.I = 0; // usually set to 0  - P controller is enough
  motor1.P_angle.D = 0; // usually set to 0  - P controller is enough
  // acceleration limit
  motor1.P_angle.output_ramp = 1e6;
  
  // motion control limits
  // angle loop velocity limit
  motor1.velocity_limit = 50;
  // either voltage limit
  motor1.voltage_limit = 12; // Volts -  default driver.voltage_limit
  // or current limit - if phase_resistance set
  motor1.current_limit = 0.2; // Amps -  default 0.5 Amps
  
  
  
  //init everything
  sensor.init();
  driver.init();
  Serial.begin(115200);
  motor1.useMonitoring(Serial);
  motor1.init();
  motor1.initFOC(0,Direction::CW);
  
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 
  motor1.monitor_downsample = 100; // default 10

  //TimerInit();
  Serial.println("Motor ready.");
  delay(1000);
}

// angle set point variable
float target_angle = 1;
// timestamp for changing direction
long timestamp_us = _micros();

void loop() {
  // put your main code here, to run repeatedly:
  motor1.monitor();
  positionControlExample();
  //motor1.loopFOC();
  //motor1.move(target);
}

void positionControlExample(){
    // each one second
  if(_micros() - timestamp_us > 1e6) {
      timestamp_us = _micros();
      // inverse angle
      target_angle = -target_angle;   
  }
  // main FOC algorithm function
  motor1.loopFOC();

  // Motion control function
  motor1.move(target_angle);
}

ISR(TIMER1_COMPA_vect){
 
  
  /*prnt(Steering_Torque);  tab;
  prnt(Atr);              tab;
  prnt(Total_Torque);     tab;*/
  /*prnt(20);               tab;
  prnt(0);                tab;
  prnt(-20);              enter;*/
}
