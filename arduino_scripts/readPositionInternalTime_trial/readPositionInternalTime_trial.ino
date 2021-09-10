
#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>
#define ledPin 13

VL53L0X sensor;
MedianFilter filterObject(28, 360);
float xn1 = 0;
float yn1 = 0;
float ynLPF = 0;
float xnLPF = 0;
int yn = 0;
int xn = 0;
int i = 0;
unsigned long mytime;
 
void TimerInit(){
  //Initialize timer1 
  //Sampling Frequency = 10 Hz
  
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

float lowPassFilter(float xnLPF) {
  
   //float yn = 0.993*yn1 + 0.003*xn + 0.003*xn1;
   //float yn = 0.969*yn1 + 0.0155*xn + 0.0155*xn1;
   ynLPF = 0.728*yn1 + 0.136*xnLPF + 0.136*xn1;
   yn1 = ynLPF;
   xn1 = xnLPF;
   return ynLPF;
}

/*void readSensor(){
  xn = sensor.readRangeSingleMillimeters();  
  yn = lowPassFilter(xn);
  Serial.print(yn);
  Serial.print("\t");
  Serial.print(xn);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("\n");
}*/

void setup() {
  pinMode(ledPin, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);  
  Wire.begin();

  sensor.setTimeout(500);
  //Serial.println(sensor.init());
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  //sensor.startContinuous();
  sensor.setMeasurementTimingBudget(50000);
  //xn1  = sensor.readRangeSingleMillimeters();
  //yn1 = sensor.readRangeSingleMillimeters();
  //ynLPF = sensor.readRangeSingleMillimeters(); 
  mytime = millis();
  TimerInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println('test');
  //delay(1);
  /*Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();*/
}

ISR(TIMER1_COMPA_vect){
  //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin
  sei();
  //i++;
  //readSensor();
  //xn = sensor.readRangeSingleMillimeters();
  xnLPF = sensor.readRangeSingleMillimeters();  
  //filterObject.in(xn);
  //yn = filterObject.out(); 
  ynLPF = lowPassFilter(xnLPF);
  mytime = millis();
  //Serial.print(yn);
  //Serial.print("\t");
  Serial.print(ynLPF);
  Serial.print("\t");
  //Serial.print(xn);
  Serial.print(xnLPF);
  Serial.print("\t");
  //Serial.print(mytime);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("\n");
  //delay(1);
}
