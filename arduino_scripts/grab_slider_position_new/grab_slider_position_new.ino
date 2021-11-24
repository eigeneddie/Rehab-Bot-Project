
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
long duration, cm;

//global variables
boolean read_position_go = false;
float current_distance;
#define interruptPin 3
long currentTime;
long startTime;
long period = 500;


void setup() {
  Serial.begin(9600);
  pinMode(pingPin, OUTPUT);
  delay(500);
  Serial.println("INIT_3: distance sensor setup complete"); Serial.println(" ");
  Serial.println("Ready when you are, Captain!");

  // setting up interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), read_position, CHANGE);
  startTime = millis();
}

void loop() {
  
  if (read_position_go == true){
    unsigned long start_program = millis();
    currentTime = millis();
    read_ultrasonic();
    Serial.print(cm);
    Serial.print(" ");
    Serial.println(currentTime-start_program);
    startTime = millis();
    unsigned long start_loop;
    
    while (read_position_go == true){
      start_loop = millis();      
      currentTime = millis();
      if (currentTime-startTime >= period){
        read_ultrasonic();
        Serial.print(cm);
        Serial.print(" ");
        Serial.print(currentTime-start_program);
        Serial.print(" ");
        Serial.println(currentTime-start_loop);
        startTime = currentTime;
      }  
    }   
  }
  
}

void read_position(){
  read_position_go = !read_position_go;
}

void read_ultrasonic(){
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH); 
  cm = microsecondsToCentimeters(duration); 
}

float microsecondsToCentimeters(long microseconds) {
   return (float) microseconds / 28.5 / 2*10;
}
