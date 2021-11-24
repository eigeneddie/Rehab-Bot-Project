
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X distance_sensor;

//global variables
boolean read_position_go = false;
float current_distance;
#define interruptPin 3
long currentTime;
long startTime;
long period = 500;


void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  distance_sensor.setTimeout(301);
  while (!distance_sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    Serial.println("Retrying...");
    delay(500);
  }
  //distance_sensor.setMeasurementTimingBudget(33000);
  Serial.println("distance sensor detected"); Serial.println(" ");
  delay(500);
  Serial.println("INIT_3: distance sensor setup complete"); Serial.println(" ");
  Serial.println("Ready when you are, Captain!");

  // setting up interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), read_position, CHANGE);
  startTime = millis();
  TimerInit();
}

void loop() {
  
  if (read_position_go == true){
    unsigned long start_program = millis();
    currentTime = millis();
    //Serial.print(distance_sensor.readRangeSingleMillimeters());
    Serial.print(distance_sensor.readRangeContinuousMillimeters());
    Serial.print(" ");
    Serial.println(currentTime-start_program);
    startTime = millis();
    
    while (read_position_go == true){      
      currentTime = millis();
      if (currentTime-startTime >= period){
        Serial.print(distance_sensor.readRangeSingleMillimeters());
        Serial.print(" ");
        Serial.println(currentTime-start_program);
        startTime = currentTime;
      }  
    }   
  }
  
}

ISR(TIMER1_COMPA_vect){
  /*sei();
  if (read_position_go == true){
    current_distance = distance_sensor.readRangeSingleMillimeters();  
    Serial.println(current_distance);
  }*/
}

void read_position(){
  read_position_go = !read_position_go;
}

void TimerInit(){
  //Initialize timer1 
  //Sampling Frequency = 2 Hz
  
  noInterrupts(); //disable all interrupts
  TCCR1A = 0; // Timer or Counter Control Register
  TCCR1B = 0; // The prescaler can be configure in TCCRx
  TCNT1  = 0; // Timer or Counter Register. The actual timer value is stored here.

  OCR1A = 31249;//31249 (2Hz);//6249(10Hz); //624 (100Hz);           // Output Compare Match Register (16Mhz/256/10Hz)
  TCCR1B |= (1 << WGM12);  // CTC (Clear Time on Compare Match)
  TCCR1B |= (1 << CS12);   // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // Interrupt Mask Register (Enable Output Compare Interrupt)
  interrupts();            // Enable all interrupts
}
