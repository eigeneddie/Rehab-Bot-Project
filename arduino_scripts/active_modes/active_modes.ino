#include <AccelStepper.h>
#include <HX711_ADC.h>
#include <VL53L0X.h>
#include <Wire.h>

/* Full-active and assistive control functionality
 * --------------------------------------------------
 * for all active exercises, there are two operation running
 * within internal interrupt:
 *  1. haptic rendering for active mode (activeGo)
 *  2. knee angle proporsional control (pidGo)
 * 
 * Active exercise split in two ways:
 *  1. assistive exercise
 *  2. full-active exercise (2 subprograms)
 *      - isotonic
 *      - isometric
 */

// 1. Defining pins: we have 3 sensors, 1 actuator
// pins
#define stepperPulse 7
#define stepperDirection 8
#define stepperEnable 9
#define HX711_dout 4 //mcu > HX711 dout pi
#define HX711_sck 5  //mcu > HX711 sck pin
//SDA_pin A4
//SCL_pin A5
//analog_in A0

// limit switches
#define interruptPin1 = 2 // front limit switch
#define interruptPin2 = 3 // rear limit switch


#define start_EMG 12 // communicate the EMG arduino to start grabbing data

// 2. initiate objects with their constructors 
AccelStepper motor_actuator(1, stepperPulse, stepperDirection);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
VL53L0X distance_sensor;

// 3. universal string command
String stringCommand;

// 4. PID variable computation 
// (although we only use the proportional term)
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
float Ts = 0.05; // 50 Hz sample frequency

// 5. Reading angle sensor and offseting
const int angleSensorPin = A0; // pot at knee mechanism
int sensorValue = 0;
const int offsetAngle = 64; // systematic offset from absolute potensiometer reading

// 6. Haptic rendering terms
float y_n, y_n_1; // position term
float a_coef_1; // position coef
float b_coef_0, b_coef_1; // force coefficient
float f_n, f_n_1; // force term
long step_target_n; // position converted to steps

// 7. Proportional control input paramters (for angle tracking)
float targetAngle; // target angle value (for single/continuous command)
float measuredAngle = 0; // value read from pot
long maxMotorSpeed; // saturation speed
float max_angle; // maximum angle limit
float min_angle; // minimum angle limit
volatile long assignedSpeed; // speed to chase knee angle

// 8. activation parameters for ISR
int count_pidGo = 0; // to down sample the proportional control
bool pidGo = false; // Go-NoGo for controller (angle tracking)
bool activeGo = false; // Go-NoGo for active admittance
bool led_state = LOW; // LED indicator

// 9. Max speeds for functions in ISR
long max_active_speed = 1000; // [step/s]
long max_return_speed = 900; // [step/s]

// 10. Knee angle monitoring 
// (undersampled and printed to serial monitor)
unsigned long startTime_dur;
unsigned long startTime; // start time
unsigned long currentTime; // current time
const unsigned long period = 500; //undersampling data period

// 11. HX711 utilities
unsigned long t = 0;
float measuredForce = 0;
boolean newDataReady = false;

// 12. VLX Distance sensor utils
float sliderDistance; // Origin is at theta = max_angle
float zeroDistance; // correcting value to sliderDistance
//float sensorReading; // actual mm reading of sensor
// note: sliderDistance = zeroDistance-sensorReading

void setup() {
  Serial.begin(9600); //Serial.setTimeout(500);
  

  Serial.println("Initiating Rehab-Bot"); Serial.println(" ");

  //=== I. Basic system functionality ===
  setup_stepper();
  
  //=== II. Load cell config ===
  setup_load_cell();
  
  //=== III. Distance Sensor config ===
  setup_distance_sensor();

  //=== IV. Internal Interrupt and Closed-loop Control Init ===
  // ->Enable PID and internal interrupt
  init_pid(Kp, Ki, Kd, N, Ts); 
  TimerInit();
}

void loop() {
  // 1. Null: Loop level, set motor speed to zero
  activeGo = false;
  motor_actuator.setSpeed(0); 
  motor_actuator.runSpeed(); // making sure the motor stops when command stops
  
  // 2. program functionality (mode '2' and '3')
  check_serial();
  
  // 3. checking knee angle
  read_angle();
  print_data("void");
}

void check_serial(){
  if (Serial.available() > 0){
    stringCommand = Serial.readStringUntil('\n');
    // if and else if statement to see if string command
    // is either '2', '3', or "-s"

    /* OPTION '2'
     * Command usage:
     * "2;ka;an;bn;bn1;max_angle;min_angle\n"
     */ 
    if (stringCommand.charAt(0) == '2'){
      // semi-assistive code goes here
    } 

    /* OPTION '3'
     * Command usage:
     * "3;0;an;bn;bn1;max_angle;min_angle\n"
     * 
     * Test system:
     * [damper spring] = [0.2 0.03] Ns/mm, N/mm, sampling_freq = 20 Hz
     * usage: 3;0;0.99252802;0.124533;0.124533;110;20
     * 
     * manually calculate other usage with 
     * other parameters using this google colab:
     * https://colab.research.google.com/drive/1NBvsuMGZIsFkjUHBi1qZHPs5PDAhqjWN#scrollTo=d69cb7a7
     * 
     */ 
    if(stringCommand.charAt(0) == '3' && stringCommand.charAt(2) == '0'){
      Serial.println("Initiating mode '3' Isotonic"); Serial.println(" ");
      pidGo = false;
      activeGo = false;
      
      // Mode '3' initialization
      // a. parse command
      Serial.println("a. parsing command"); Serial.println(" ");
      delay(500);
      parse_active_isotonic(stringCommand); // parsing single command

      // b. zero proportional control
      Serial.println("b. zeroing control param"); Serial.println(" ");
      delay(500);
      zero_everything();
      
      // c. zeroing stepper position (home)
      Serial.println("c. zeroing motor position"); Serial.println(" ");
      delay(500);
      read_angle();
      back_to_flexion();
      motor_actuator.setCurrentPosition(0); // set home

      // d. zeroing load cell (tare)
      Serial.println("d. zeroing force sensor"); Serial.println(" ");
      LoadCell.tare(); // tare (zero load cell)
      boolean tareStatus = false;
      while(!tareStatus){
        
        Serial.println("taring");
        delay(1000);
        if(LoadCell.getTareStatus() == true){
          tareStatus = true;
        }
        Serial.println(tareStatus);
      }
      Serial.println("Tare complete."); Serial.println(" ");
      delay(500);

      // e. slider position
      distance_sensor.setMeasurementTimingBudget(200000); //set measurements for high accuracy
      zeroDistance = distance_sensor.readRangeSingleMillimeters(); //read position once
      sliderDistance = zeroDistance - distance_sensor.readRangeSingleMillimeters(); //current slider position from origin
      distance_sensor.setMeasurementTimingBudget(50000); //set measurements for high speed (takes up to 50 ms on loop)
      //distance_sensor.startContinuous(); //read distance continuously

      // f. enter program
      Serial.println("Entering mode '3': isotonic");
      delay(500);
      activeGo = true;

      startTime = millis();
      long startTime_local = millis();

      digitalWrite(start_EMG, HIGH); // start getting EMG data!
      print_data_once("mode30");

      while(activeGo){ // EXECUTE ACTIVE TRAINING MODE
        long start_loop = millis();
        // a. Read force sensor
        read_force();

        // b. move motor
        move_motor();

        // c. Current measured angle
        read_angle();

        // d. Current measure slider position
        read_slider_position();
        
        // e. Stoping criteria (force stop "-s")
        stopping_criteria_hap_ren();
        
        // e. undersampling print
        print_data("mode30");

        // f. check if measured angle <= min_angle
        back_to_flexion_2();

        // g. tare force sensor when received 't' command
        tare_force_sensor();
        currentTime = millis();

        // h. print loop duration time
        if (currentTime-startTime_local >= period){
          Serial.println(currentTime-start_loop);
          startTime_local = currentTime;
        }
      }
      digitalWrite(start_EMG, LOW);
    }
  } 
}

// FUNCTIONS X: Setup functions
//------------------------------
// setup stepper motor pins and other pins
void setup_stepper(){
  pinMode(stepperPulse, OUTPUT);
  pinMode(stepperDirection, OUTPUT);
  pinMode(stepperEnable, OUTPUT);

  digitalWrite(stepperPulse, LOW);
  digitalWrite(stepperDirection, LOW);
  digitalWrite(stepperEnable, HIGH);
  motor_actuator.setAcceleration(1000);

  pinMode(start_EMG, OUTPUT);
  digitalWrite(start_EMG, LOW);
  //=== Limit switch Interrupt ===
  /*pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), limit_switch, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), limit_switch, RISING);*/
  //=================================

  Serial.println("INIT_1: Stepper motor setup complete"); Serial.println(" ");
}

// setup HX711 for load cell/force reading
void setup_load_cell(){
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 98.35;//94.83;//104.46;// for beam load cell (219.0 is the value for the s-type load cell at hand)
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  
  while(LoadCell.getTareTimeoutFlag()){
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    Serial.println("Retrying...");
    delay(200);
  }
  LoadCell.setCalFactor(calibrationValue);
  Serial.println("INIT_2: Load cell setup complete"); Serial.println(" ");
}

// setup VL53L0X distance sensor
void setup_distance_sensor(){
  Wire.begin();
  distance_sensor.setTimeout(50);
  
  while (!distance_sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    Serial.println("Retrying...");
    delay(500);
  }
  Serial.println("distance sensor detected"); Serial.println(" ");
  delay(500);

  //Note: to get distance, use: 
  // distance_sensor.readRangeSingleMillimeters();
  // distance_sensor.startContinuous();
  // distance_sensor.stopContinuous();
  // distance_sensor.readRangeContinuousMillimeters(); -->
  //sensor.setMeasurementTimingBudget(100000); --> change accuracy
  Serial.println("INIT_3: distance sensor setup complete"); Serial.println(" ");
}

// FUNCTIONS I: Parsing commands
//------------------------------------------
// I.a. Active mode '2': Assistive control 
void assistive_control (String command_data){
  String acoef1 = getValue(command_data, ';', 2);
  String bcoef0 = getValue(command_data, ';', 3);
  String bcoef1 = getValue(command_data, ';', 4);
  String maxAng = getValue(command_data, ';', 5);
  String minAng = getValue(command_data, ';', 6);

  a_coef_1 = acoef1.toFloat();
  b_coef_0 = bcoef0.toFloat();
  b_coef_1 = bcoef1.toFloat();
  max_angle = maxAng.toFloat();
  min_angle = minAng.toFloat();
}

// I.b. Active mode '3' & '0': Isotonic training
void parse_active_isotonic (String command_data){
  //3;0;0.99252802;0.124533;0.124533;110;20
  String acoef1 = getValue(command_data, ';', 2);
  String bcoef0 = getValue(command_data, ';', 3);
  String bcoef1 = getValue(command_data, ';', 4);
  String maxAng = getValue(command_data, ';', 5);
  String minAng = getValue(command_data, ';', 6);

  a_coef_1 = acoef1.toFloat();
  b_coef_0 = bcoef0.toFloat();
  b_coef_1 = bcoef1.toFloat();
  max_angle = maxAng.toFloat();
  min_angle = minAng.toFloat();
}

// I.c. Active mode '3' & '1': Isometric training
void parse_active_isometric (String command_data){
  String acoef1 = getValue(command_data, ';', 2);
  String bcoef0 = getValue(command_data, ';', 3);
  String bcoef1 = getValue(command_data, ';', 4);
  String maxAng = getValue(command_data, ';', 5);
  String minAng = getValue(command_data, ';', 6);

  a_coef_1 = acoef1.toFloat();
  b_coef_0 = bcoef0.toFloat();
  b_coef_1 = bcoef1.toFloat();
  max_angle = maxAng.toFloat();
  min_angle = minAng.toFloat();
}

// I.d parsing string commands
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

// FUNCTIONS II: Return to flexion position
//------------------------------------------
// II.a. at init, return knee angle to flexion (max_angle) position 
// using proportional control.
void back_to_flexion(){
  if (measuredAngle != max_angle){
    maxMotorSpeed = max_return_speed;
    motor_actuator.setMaxSpeed(maxMotorSpeed);
    pidGo = true;
    activeGo = false;
    while(pidGo){
      // i. measure angle
      read_angle();

      // ii. set target angle
      targetAngle = max_angle;

      // iii. run motor
      motor_actuator.setSpeed(assignedSpeed);
      motor_actuator.runSpeed();
      
      // iv. print data
      print_data("back to flexion");
      stopping_criteria_angleCont();
    }
  }
}

// II.b. during training and at min_angle, 
//       return knee angle to stepper home position (use homing)
void back_to_flexion_2(){
  if (measuredAngle <= min_angle){
    maxMotorSpeed = max_return_speed;
    boolean home_pos = false;
    activeGo = false;
    zero_everything();
    while(!home_pos){
      // i run motor
      motor_actuator.setMaxSpeed(maxMotorSpeed);
      motor_actuator.moveTo(0);
      motor_actuator.run();
      
      if (motor_actuator.currentPosition() == 0){
        home_pos = true;
        activeGo = true;
      }
      // iii. print data
      print_data("back to flexion 2");
    }
  }
}

// FUNCTIONS III: Read sensors, store, and run motor
//------------------------------------------
// III.a move the motor to target position
void move_motor(){
  motor_actuator.setMaxSpeed(max_active_speed);
  motor_actuator.moveTo(-step_target_n); // It's minus because our positive
  motor_actuator.run();                  // and the machine's positive is different.
}                                        // This is not some sort of a quick fix, it's just logic.

// III.b read force data
void read_force(){

  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t) {
      measuredForce = LoadCell.getData()/1000*9.81;
      if (measuredForce < 0 || measuredForce < 0.2){
        measuredForce = 0.00;
      }
      newDataReady = 0;
      t = millis();
    }
  }
}

// III.c re-tare force sensor
void tare_force_sensor(){
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tare();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

// III.d read current knee angle
void read_angle(){
  sensorValue = analogRead(angleSensorPin);
  measuredAngle = float(map(sensorValue, 0, 1023, 0, 333)-offsetAngle);
}

// III.e read current slider position
void read_slider_position(){
  //sliderDistance = zeroDistance - distance_sensor.readRangeContinuousMillimeters();
  sliderDistance = zeroDistance - distance_sensor.readRangeSingleMillimeters();
}

// III.f print current data to screen/send to high-level controller
// WARNING, printing at higher frequencies can degrade control performance
void print_data(String mode){
  currentTime = millis(); // grab time
  if (currentTime-startTime >= period){
    Serial.print(mode); Serial.print(" ");
    Serial.print(measuredAngle); Serial.print(" ");
    Serial.print(sliderDistance); Serial.print(" ");
    Serial.print((float) step_target_n/50); Serial.print(" ");
    Serial.print(measuredForce); Serial.println(" ");
    startTime = currentTime;
  }  
}

void print_data_once(String mode){
  Serial.print(mode); Serial.print(" ");
  Serial.print(measuredAngle); Serial.print(" ");
  Serial.print(sliderDistance); Serial.print(" ");
  Serial.print((float) step_target_n/50); Serial.print(" ");
  Serial.print(measuredForce); Serial.println(" ");
}

// FUNCTIONS IV: Main control functions (inside Internal Interrupt, TIMER1)
//------------------------------------------
// IV.a ISR for executing haptic rendering & proportional control
ISR(TIMER1_COMPA_vect){
  //====== proportional control =======
  if (pidGo == true){
    count_pidGo++;
    if(count_pidGo>19){
      assignedSpeed = int(pid_execute(targetAngle, measuredAngle, maxMotorSpeed));
      count_pidGo = 0;
    }
    digitalWrite(LED_BUILTIN, LOW);
  } 
  else {
    assignedSpeed = 0;
  }
  //==================================

  //====== haptic rendering =======
  if (activeGo == true && pidGo == false){
    step_target_n = haptic_rendering(measuredForce);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  //==================================
}

// IV.b zero every control parameters (haptic rendering and proportional control)
void zero_everything(){
  e2 = 0, e1 = 0, e0 =0;
  u2 = 0, u1 = 0, u0 = 0; 
  f_n = 0, f_n_1 = 0;
  y_n = 0, y_n_1 = 0;
  measuredForce = 0.0;
  step_target_n = 0;
}

// IV.c Resistance Algorithm (admittance rendering) 
float haptic_rendering(float measured_force){
  f_n = measured_force;
  y_n = a_coef_1*y_n_1 + b_coef_0*f_n + b_coef_1*f_n_1;
  long position_target = (long) y_n*50.0; // (1/8 rev/mm) * (400 steps/rev) 
  f_n_1 = f_n;
  y_n_1 = y_n;
  return position_target;
}

// IV.d initialize proportional control
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

// IV.e proportional control execution
float pid_execute(float target_angle, float plant_output, long speed_sat){
  /* Executing PID control algorithm
   * 
   *  Args:
   *    target_angle [float]: reference angle [deg]
   *    plant_output [float]: measured angle [deg]
   *    speed_sat [float]: max/min speed of motor [pulse/s] (aka speed saturation)
   */

  /*e2 = e1; 
  e1 = e0; 
  u2 = u1;
  u1 = u0;*/

  r = target_angle;
  y = plant_output;

  float speed_sat_fl = float(speed_sat);
  e0 = r-y; // compute error
  u0 = ke0*e0; //Kp*e0;//-ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2; 

  if (u0 >= speed_sat_fl){
    u0 = speed_sat;
  }

  else if (u0 <= -speed_sat_fl){
    u0 = -speed_sat;
  }
  return u0;
}

// IV.f Customizes internal interrupt counters
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

  OCR1A = 3124;//1249; // Output Compare Match Register (16Mhz/256/<sampling_freq>Hz)
  //62499 (1 Hz);//31249 (2Hz); //15625 (4Hz) //6249 (10Hz); //1249 (50Hz);
  //624 (100Hz); 

  TCCR1B |= (1 << WGM12);  // CTC (Clear Time on Compare Match)
  TCCR1B |= (1 << CS12);   // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // Interrupt Mask Register (Enable Output Compare Interrupt)
  interrupts();            // Enable all interrupts
}

// FUNCTIONS V: loop stopping criteria
//------------------------------------------
// V.a stopping criteria for angle tracking (proportional control)
void stopping_criteria_angleCont(){
  if (Serial.available() > 0){
    stringCommand = Serial.readStringUntil('\n');
    if (stringCommand == "-s"){
      pidGo = false;
      activeGo = true;
    }
  }
  
  if (measuredAngle >= max_angle){ // when exceeding max_angle, disable proportional control
    pidGo = false;
    activeGo = true;
  }
}

// V.b stopping criteria for haptic rendering
void stopping_criteria_hap_ren(){
  if (Serial.available() > 0){
    stringCommand = Serial.readStringUntil('\n');
    if (stringCommand == "-s"){
      pidGo = false;
      activeGo = false;
    }
  }
  
  if (measuredAngle >= max_angle){ // when exceeding max_angle, disable proportional control 
    pidGo = false;
    activeGo = true;
  }
}

// External interrupt - limit switch
void limit_switch(){
  //Hardware safety @software level: forcefully stop the driver when 
  //the slider happens to reach the absolute endz of the rail
  digitalWrite(stepperEnable, LOW);

}
