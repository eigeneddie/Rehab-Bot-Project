/* Rehabilitation Machine display script
 *  
 *  created by Edgar B. Sutawika
 *  
 *  Rehabilitation project for lower extremity stroke patients.
 *  
 *  Utilizing codes:
 *  Graphics library by ladyada/adafruit with init code from Rossum
 *  SPFD5408 library by Joao Lopes
 *  Template following Henning Karlsen from HowToMechatronics
 *  
 ============= ==========*/

 /*
  * Quick note:
  * 
  * Rehab-bot modes
  * 
  * 1. Passive: not written yet (20210901)
  *   Passive control is done mostly within the arduino module (low-level module)
  *   High level module (raspberry pi) only computes and sends out control parameters for arduino to execute.
  *   Arduino does a predetermined trajectory routine in a certain amount of time.
  *   
  *   key working of passive mode: 
  *     a. 
  *   
  *   main menu for passive control:
  *     set current angle (controlled live from LCD TFT with a preset speed)
  *     max knee angle
  *     min knee angle
  *     speed duration --> set Constant speed in stepper accel
  *     duration
  *    
  *     
  *     For passive mode, there's a lot of communication between device and User interface
  *     
  *     
  *     When the routine begins, the device goes to minimum angle, and starts the routine
  *     
  *     
  * ===========================================
  * for active control, activation code is as follow:
  *   current page number - sub-option 1 - sub-option 2 
  * 
  * 2. Semi-assistive: 
  *   current_page_number - assistive constants (3 options) - spring damper config (3 options)
  *   2 - 0 - 0 
  *   2 - 0 - 1
  *   2 - 0 - 2
  *   2 - 1 - 0
  *   2 - 1 - 1
  *   2 - 1 - 2
  *   2 - 2 - 0
  *   2 - 2 - 1
  *   2 - 2 - 2
  *   
  *   (9 options)
  *   
  * 3. Full-active:
  *   current_page_number - ISOMETRIC/ISOTONIC (2 options) - spring damper config (3 options)
  *   3 - 0 - 0 
  *   3 - 0 - 1
  *   3 - 0 - 2
  *   3 - 1 - 0
  *   3 - 1 - 1
  *   3 - 1 - 2
  * 
  *   (6 options)
  * 
  */

//=====Critical libraries=========//
#include <SPFD5408_Adafruit_GFX.h>
#include <SPFD5408_Adafruit_TFTLCD.h>
#include <SPFD5408_TouchScreen.h>

//===Other critical setup, but ===//
//===I don't know how it works ===//
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0xD0D0

//======== The pins =========//
//== and calibrated values ==//
#define YP A2
#define XM A1
#define YM 6
#define XP 7

#define TS_MINX 920
#define TS_MINY 99
#define TS_MAXX 170
#define TS_MAXY 822

//==== initiate touchscreen and    ==//
//==== tft lcd functionality class ==//
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define boxWidth  200
#define boxHeight 35

#define boxWidthMM  140 //Box width mode menu
#define boxHeightMM 35 //Box height mode menu
#define butIncr     12 // space between button 
#define butIncrBig  70 // space between button (bigger)
#define sqrButt     35 // square button size
#define MINPRESSURE 10
#define MAXPRESSURE 1000

//Define and initiate mode configuration

//Training mode
char currentPage = 0; // 4 option (0, 1 (passive), 2 (semi-active), 3 (full-active))

//=========IMPORTANT=============
//===MODE SPECIFIC ACTIVATION====
//============CODE===============

//Detail passive mode (in their initial condition)
//------------------------------------------------
// activation code = 10+1 = 11 digits
// WARNING: KNEE ANGLE --> 123 deg (bengkok) to 0 deg (lurus)
int currentKneeAngle = 70;// IC will be sent from the arduino-> raspberry pi->LCD
int maxKneeAngle = 100; // Default max angle [deg] range: 90-170 (3 digit)
int minKneeAngle = 10; // Default min angle [deg] range: 10-89 (2 digit)
int rehabSpeed = 10; // Default rehab speed [%] range: 10-100 (3 digit)
int duration = 1; // default duration [minutes] range: 1-60 (2 digit)

int maxLimit = 123;
int minLimit = 0;

//Detail semi-assistive mode
//------------------------------------------------
// activation code = 2+1 = 3 digits
int assistConst = 0; // 3 option (can be increased/decreased)
int admittance1 = 0; // 3 option (can be increased/decreased)

int options_semi_active = 3; // not exactly an important variable

//Detail full active mode
//------------------------------------------------
// activation code = 2+1 = 3 digits
int activeModeVar = 0; // 2 option (isotonic - isometric)
int admittance2 = 0; // 3 option (can be increased/decreased)

int options_Active = 3; // not exactly an important variable either

// execution code arrays
int exeCodePassive[11];
int exeCodeSemiActive[3];
int exeCodeActive[3];

//===page activation====
bool activation_passive = false;
bool activation_semi_active = false;
bool activation_full_active = false;
String activationCode;

void setup() {
  Serial.begin(9600);
  tft.reset();
  tft.begin(0x9341);
  drawHomeScreen();
}

// main loop
void loop() {
  
  TSPoint p = ts.getPoint(); // pressure object
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  //Main loop deals with pressure location
  //Custom functions deal with drawing the UI
  
  //======PAGE 0: HOME SCREEN=========
  homePageMenu(p);
    
  //======PAGE 1: PASSIVE MODE=========
  passiveModeMenu(p);

  //======PAGE 2: SEMI-ACTIVE MODE=========
  semiActiveModeMenu(p);

  //======PAGE 3: FULL ACTIVE MODE=========
  fullActiveModeMenu(p); 

}

// ====== Custom Funtions ======
void readXYLocation(TSPoint p){
  /* inspection function*/
    Serial.print("("); Serial.print(p.x);
    Serial.print(", "); Serial.print(p.y);
    Serial.println(")");
}

//===========PRESSURE============
//===FUNCTIONS USER INTERFACE====
//===============================
void homePageMenu(TSPoint p){
  if(currentPage == 0){
    if (p.z < MAXPRESSURE && p.z > MINPRESSURE) { 
      p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
      p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
      //readXYLocation(p);
  
      // ---a. If selected passive mode
      if ( (p.x > 15) && (p.x < 215) && (p.y > 199 ) && (p.y < 234)) {
          drawFrame(20, 90, p);
          currentPage = 1;
          delay(20);
          drawPassiveMode();
      } // end if passive
  
      // ---b. If selected semi-assistive mode
      if ((p.x > 15) && (p.x < 215) && (p.y > 125 ) && (p.y < 160)) {
         
         drawFrame(20, 160, p);
         currentPage = 2;
         delay(20);
         drawSemiActiveMode();
      } // end if semi-assistive
  
      // ---c. If selected active mode
      if ( (p.x > 15) && (p.x < 215) && (p.y > 56 ) && (p.y < 91)) {
          drawFrame(20, 230, p);
          currentPage = 3;
          delay(100);
          drawActiveMode();
       } // end if active
  
      } //   
    }
}

void passiveModeMenu(TSPoint p){
  /* Passive mode main menu UI.
   *-> Six menu buttons
   * a. Back
   * b. Current position
   * c. Max
   * d. Min
   * e. Speed
   * f. Duration
   * g. start
   * h. stop */

  // Current page 1 = passive mode
  if(currentPage == 1){

    if (p.z < MAXPRESSURE && p.z > MINPRESSURE) { 
      p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
      p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
      readXYLocation(p);

      // a. Back button
      // ------------------------------------------------
      if ((p.x > 19) && (p.x < 217) && (p.y > 260 ) && (p.y < 292) ) {//
        drawFrame(20, 30, p);
        currentPage = 0;
        delay(20);
        drawHomeScreen();
        if (activation_passive==true){
          activation_passive == false;
        } 
      } 
  
      // b. LEFT ARROW - CURRENT KNEE ANGLE - RIGHT ARROW
      // ------------------------------------------------
      // LEFT
      if ((p.x > 193) && (p.x < 218) && (p.y > 220 ) && (p.y < 243)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5), p);
        currentKneeAngle--;
        if(currentKneeAngle<minLimit){
          currentKneeAngle = maxLimit;
        }
        redraw(50, 30+(boxHeightMM+5), currentKneeAngle-1, "current:");
        Serial.println("0;" + String(currentKneeAngle));
      } 
      
      // RIGHT
      if ((p.x > 17) && (p.x < 43) && (p.y > 220 ) && (p.y < 243)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5), p);
        currentKneeAngle++;
        if(currentKneeAngle>=maxLimit){
          currentKneeAngle = minLimit;
        }
        redraw(50, 30+(boxHeightMM+5), currentKneeAngle-1, "current:");
        Serial.println("0;" + String(currentKneeAngle));
      }
      
      //c. LEFT ARROW - MAX KNEE ANGLE - RIGHT ARROW
      // ------------------------------------------------
      // LEFT
      if ((p.x > 193) && (p.x < 218) && (p.y > 176 ) && (p.y < 203)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*2, p);
        maxKneeAngle--;
        if(maxKneeAngle<30){
          maxKneeAngle = maxLimit;
          }
        redraw(50, 30+(boxHeightMM+5)*2, maxKneeAngle-1, "max:");
      } 
      
      // RIGHT
      if ((p.x > 17) && (p.x < 43) && (p.y > 176 ) && (p.y < 203)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*2, p);
        maxKneeAngle++;
        if(maxKneeAngle>maxLimit){
          maxKneeAngle = 30;
          }
        redraw(50, 30+(boxHeightMM+5)*2, maxKneeAngle-1, "max:");
      }

      //d. LEFT ARROW - MIN KNEE ANGLE - RIGHT ARROW
      if ((p.x > 193) && (p.x < 218) && (p.y > 144) && (p.y < 162)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*3, p);
        minKneeAngle--;
        if(minKneeAngle<minLimit){
          maxKneeAngle = 29;
        }
        redraw(50, 30+(boxHeightMM+5)*3, minKneeAngle-1, "min:");
      } 
        
      if ((p.x > 17) && (p.x < 43) && (p.y > 144) && (p.y < 162)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*3, p);
        minKneeAngle++;
        if(minKneeAngle>29){
          minKneeAngle = minLimit;
        }
        redraw(50, 30+(boxHeightMM+5)*3, minKneeAngle-1, "min:");
      }

      //e. LEFT ARROW - REHAB SPEED - RIGHT ARROW
      if ((p.x > 193) && (p.x < 218) && (p.y > 95) && (p.y < 120)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*4, p);
        rehabSpeed = rehabSpeed - 10;
        if(rehabSpeed<10){
          rehabSpeed = 100;
        }
        redraw(50, 30+(boxHeightMM+5)*4, rehabSpeed-1, "speed:");
      } 
        
      if ((p.x > 17) && (p.x < 43) && (p.y > 95 ) && (p.y < 120)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*4, p);
        rehabSpeed = rehabSpeed + 10;
        if(rehabSpeed>100){
          rehabSpeed = 10;
        }
        redraw(50, 30+(boxHeightMM+5)*4, rehabSpeed-1, "speed:");
      }

      //f. LEFT ARROW - DURATION - RIGHT ARROW
      if ((p.x > 193) && (p.x < 218) && (p.y > 53) && (p.y < 76)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*5, p);
        duration--;
        if(duration<1){
          duration = 60;
        }
        redraw(50, 30+(boxHeightMM+5)*5, duration-1, "dur:");
      } 
        
      if ((p.x > 17) && (p.x < 43) && (p.y > 53 ) && (p.y < 76)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 30+(boxHeightMM+5)*5, p);
        duration++;
        if(duration>60){
          duration = 1;
        }
        redraw(50, 30+(boxHeightMM+5)*5,duration-1, "dur:");
      }

      //g. Start button sequence
      if ((p.x > 146) && (p.x < 200) && (p.y > 14 ) && (p.y < 40)){
        startButtonENGAGED();
        activation_passive = true;
        delay(200);
        String activationCodePassive1 = "1;" + String(maxKneeAngle) + ';' + String(minKneeAngle);
        String activationCodePassive2 =  ';' + String(rehabSpeed) + ';' + String(duration) + "\n";
        String activationCodePassive = activationCodePassive1 + activationCodePassive2;
        Serial.println(activationCodePassive);
      }
      
      //h. Stop button sequence
      if ((p.x > 25) && (p.x < 88) && (p.y > 17 ) && (p.y < 45)){
        if (activation_passive == true){
          startButton();
          stopButtonENGAGED();
          activation_passive = false;
          Serial.println("-s");
          delay(200);
          stopButton();          
        }
  
      }  
    }
  
  }

}

void semiActiveModeMenu(TSPoint p){

  //int assistConst = 0; // 3 option (can be increased/decreased)
  //int admittance1 = 0; // 3 option (can be increased/decreased)
  //int options_semi_active = 3;
  if(currentPage == 2){
    
    if (p.z < MAXPRESSURE && p.z > MINPRESSURE) { 
      p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
      p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
      //readXYLocation(p);
    
      // a. Back button
      if ((p.x > 19) && (p.x < 215) && (p.y > 247 ) && (p.y < 274) ) {//
        drawFrame(20, 50, p);
        currentPage = 0;
        delay(20);
        drawHomeScreen();
        if (activation_semi_active==true){
          activation_semi_active == false;
        } 
      } 

      // b. LEFT ARROW - THREE ASSISTIVE CONSTANTS - RIGHT ARROW    
      if ( (p.x > 188) && (p.x < 219) && (p.y > 191 ) && (p.y < 220)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
        drawFrameSmall(50, 50+(boxHeightMM+butIncr), p);
        assistConst--;
        if(assistConst<0){assistConst = 2;}
        redraw(50, 50+(boxHeightMM+butIncr), assistConst, "CONST.");
      } 
        
      if (( p.x > 19) && (p.x < 42) && (p.y > 191 ) && (p.y < 220)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
         drawFrameSmall(50, 50+(boxHeightMM+butIncr), p);
         assistConst++;
         if(assistConst>=options_semi_active){assistConst = 0;}
         redraw(50, 50+(boxHeightMM+butIncr), assistConst, "CONST.");
      }
      
      // c. LEFT ARROW - ADMITTANCE x - RIGHT ARROW
      if ( (p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
         drawFrameSmall(50, 50+(boxHeightMM+butIncr)*2, p);
         admittance1--;
         if(admittance1<0){admittance1 = 2;}
         redraw(50, 50+(boxHeightMM+butIncr)*2, admittance1, "ENV.");
      } 
      
      if ( (p.x > 14) && (p.x < 38) && (p.y > 141 ) && (p.y < 167)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
         drawFrameSmall(50, 50+(boxHeightMM+butIncr)*2, p);
         admittance1++;
         if(admittance1>=options_semi_active){admittance1 = 0;}
         redraw(50, 50+(boxHeightMM+butIncr)*2, admittance1, "ENV.");
      }

      // d. Start button 
      if ((p.x > 146) && (p.x < 200) && (p.y > 14 ) && (p.y < 40)){
        startButtonENGAGED();
        activation_semi_active = true;
        delay(200);
        exeCodeSemiActive[0] = currentPage;
        exeCodeSemiActive[1] = assistConst;
        exeCodeSemiActive[2] = admittance1;
        activationCode = generateActivationCodeString(exeCodeSemiActive[0],exeCodeSemiActive[1],exeCodeSemiActive[2]);
        Serial.println(activationCode);
      }

      // e. STOP button
      if ((p.x > 25) && (p.x < 88) && (p.y > 17 ) && (p.y < 45)){
        if (activation_semi_active == true){
          startButton();
          stopButtonENGAGED();
          activation_semi_active = false;
          Serial.println("-s");
          delay(200);
          stopButton();          
        }
      }
            
    } //
  }
}

void fullActiveModeMenu(TSPoint p){
   if(currentPage == 3){
    
      if (p.z < MAXPRESSURE && p.z > MINPRESSURE) { 
        p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
        p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
        //readXYLocation(p);
    
        // a. Back button
        if ((p.x > 19) && (p.x < 215) && (p.y > 247 ) && (p.y < 274) ) {//
            drawFrame(20, 50, p);
            currentPage = 0;
            delay(20);
            drawHomeScreen();
            if (activation_full_active==true){
              activation_full_active == false;
            } 
         }
  
      // b. LEFT ARROW - ISOTONIC - ISOMETRIC - RIGHT ARROW
        if ((p.x > 188) && (p.x < 219) && (p.y > 191 ) && (p.y < 220) ||((p.x > 19) && (p.x < 42) && (p.y > 191 ) && (p.y < 220))  ) { //
           drawFrameSmall(50, 50+(boxHeightMM+butIncr), p);
           if (activeModeVar == 0){
             activeModeVar = 1;
             singleBlueLeftRightButton(50, 50+(boxHeightMM+butIncr), "ISOMETRIC");
           }
           else if (activeModeVar == 1){
             activeModeVar = 0;
             singleBlueLeftRightButton(50, 50+(boxHeightMM+butIncr), "ISOTONIC");
           }
           delay(20);
        } 
      
      // c. LEFT ARROW - ADMITTANCE x - RIGHT ARROW
        if ( (p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
           drawFrameSmall(50, 50+(boxHeightMM+butIncr)*2, p);
           admittance2--;
           if(admittance2<0){admittance2 = 2;}
           redraw(50, 50+(boxHeightMM+butIncr)*2, admittance2, "ENV.");
        } 
        
        if ( (p.x > 14) && (p.x < 38) && (p.y > 141 ) && (p.y < 167)) { //(p.x > 196) && (p.x < 215) && (p.y > 141 ) && (p.y < 176)
           drawFrameSmall(50, 50+(boxHeightMM+butIncr)*2, p);
           admittance2++;
           if(admittance2>=options_Active){admittance2 = 0;}
           redraw(50, 50+(boxHeightMM+butIncr)*2, admittance2, "ENV.");
        }

        //d. Start & Stop button sequence
        if ((p.x > 146) && (p.x < 200) && (p.y > 14 ) && (p.y < 40)){
          startButtonENGAGED();
          activation_full_active = true;
          delay(200);
          exeCodeActive[0] = currentPage;
          exeCodeActive[1] = activeModeVar;
          exeCodeActive[2] = admittance2;
          activationCode = generateActivationCodeString(exeCodeActive[0],exeCodeActive[1],exeCodeActive[2]);
          Serial.println(activationCode);
        }
        
        if ((p.x > 25) && (p.x < 88) && (p.y > 17 ) && (p.y < 45)){

          if (activation_full_active == true){
            startButton();
            stopButtonENGAGED();
            activation_full_active = false;
            Serial.println("-s");
            delay(200);
            stopButton();          
          }
   
        }
              
        
      } //
    }
}


//===========BASIC===============
//====MAIN DRAWING FUNCTIONS=====
//===============================

void drawHomeScreen() {
  // General
  tft.setTextColor(RED);
  tft.setTextSize(3);
  tft.fillScreen(BLACK);

  // Title
  tft.setCursor(20, 20);
  tft.println("SELECT MODE");
  tft.drawFastHLine(50, 50, 140, WHITE);
  tft.setTextSize(1);
  tft.setCursor(57, 60);
  tft.println("Biomechanics Lab. ITB");

  // First button
  singleBlueButton(20, 90, "PASSIVE");

  // Second button
  singleBlueButton(20, 90+butIncrBig, "SEMI-ASSISTIVE");

  // Third button
  singleBlueButton(20, 90+butIncrBig*2, "FULL ACTIVE");

  // Author
  authorSign(5, 305, "2021", "Edgar B. Sutawika");

}

// Highlights the button when pressed
void drawFrame(int x1, int y1, TSPoint p) { // drawing coordinate
  
  tft.drawFastHLine(x1, y1, boxWidth, RED); //Top
  tft.drawFastHLine(x1, y1+boxHeight, boxWidth, RED); //Bottom
  tft.drawFastVLine(x1, y1, boxHeight, RED); //Left
  tft.drawFastVLine(x1+boxWidth, y1, boxHeight, RED); //Right
  
  delay(300);
  
  tft.drawFastHLine(x1, y1, boxWidth, BLUE); //Top
  tft.drawFastHLine(x1, y1+boxHeight, boxWidth, BLUE); //Bottom
  tft.drawFastVLine(x1, y1, boxHeight, BLUE); //Left
  tft.drawFastVLine(x1+boxWidth, y1, boxHeight, BLUE); //Right
}

void drawFrameSmall(int x1, int y1, TSPoint p) { // drawing coordinate
  
  tft.drawFastHLine(x1, y1, boxWidthMM, RED); //Top
  tft.drawFastHLine(x1, y1+boxHeightMM, boxWidthMM, RED); //Bottom
  tft.drawFastVLine(x1, y1, boxHeightMM, RED); //Left
  tft.drawFastVLine(x1+boxWidthMM, y1, boxHeightMM, RED); //Right
  
  delay(300);

  tft.drawFastHLine(x1, y1, boxWidthMM, BLUE); //Top
  tft.drawFastHLine(x1, y1+boxHeightMM, boxWidthMM, BLUE); //Bottom
  tft.drawFastVLine(x1, y1, boxHeightMM, BLUE); //Left
  tft.drawFastVLine(x1+boxWidthMM, y1, boxHeightMM, BLUE); //Right
}


//===============================
//====DRAWING PROGRAM MODES======
//===============================

void drawPassiveMode(){
  // a. General
  tft.fillScreen(BLACK);
  
  // b. Title page
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.setCursor(55, 10);
  tft.println("PASSIVE MODE");
  tft.drawFastHLine(50, 27, 145, WHITE);

  // c. Back button
  singleBlueButton(20, 30, "BACK");
  
  // d. First button: Current Knee Angle
  singleBlueLeftRightButton(50, 30+(boxHeightMM+5), "Current:"+ String(currentKneeAngle));
  //"Current: "+currentKneeAngle
  // e. Second button: Max Knee Angle
  singleBlueLeftRightButton(50, 30+(boxHeightMM+5)*2, "Max: "+ String(maxKneeAngle));

  //f. third button: Min Knee angle
  singleBlueLeftRightButton(50, 30+(boxHeightMM+5)*3, "Min: "+ String(minKneeAngle));

  //g. fourth button: rehab speed
  singleBlueLeftRightButton(50, 30+(boxHeightMM+5)*4, "Speed: "+ String(rehabSpeed));

  //f. fifth button: Min Knee angle
  singleBlueLeftRightButton(50, 30+(boxHeightMM+5)*5, "dur:"+ String(duration));
  
  // g. Start button
  startButton();

  // h. Stop button
  stopButton();
     
}

void drawSemiActiveMode(){
  // a. General
  tft.fillScreen(BLACK);
  
  // b. Title page
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.setCursor(20, 10);
  tft.println("SEMI ACTIVE MODE");
  tft.drawFastHLine(50, 30, 145, WHITE);

  // c. Back button
  singleBlueButton(20, 50, "BACK");
  
  // d. First button
  singleBlueLeftRightButton(50, 50+(boxHeightMM+butIncr), "CONST. "+String(assistConst+1));

  // e. Second button
  singleBlueLeftRightButton(50, 50+(boxHeightMM+butIncr)*2, "ENV. "+String(admittance1+1));
  //singleBlueButton(20, 50+(boxHeightMM+butIncr)*2, "ADMITT. 1");

  // g. Start button
  startButton();

  // h. Stop button
  stopButton();
   
}

void drawActiveMode(){
  // a. General
  tft.fillScreen(BLACK);
  
  // b. Title page
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.setCursor(60, 10);
  tft.println("ACTIVE MODE");
  tft.drawFastHLine(50, 30, 145, WHITE);

  // c. Back button
  singleBlueButton(20, 50, "BACK");
  
  // d. First button
  singleBlueLeftRightButton(50, 50+(boxHeightMM+butIncr), "ISOTONIC");

  // e. Second button
  singleBlueLeftRightButton(50, 50+(boxHeightMM+butIncr)*2, "ENV. "+ String(admittance2+1));
  //singleBlueButton(20, 50+(boxHeightMM+butIncr)*2, "ADMITT. 1");

  // g. Start button
  startButton();

  // h. Stop button
  stopButton();
 
}


//===========OTHER===============
//====FUNCTIONALITY BUTTONS======
//===============================

void singleBlueButton(int x_or, int y_or, String buttonTitle){ // 
   /* IMPORTANT BUTTON: SINGULAR button that only has ONE OPTION. 
    Arg:
      x_or: x location of button
      y_or: y location of button
      buttonTitle: parameter name for button
  */   
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.fillRect(x_or, y_or, boxWidth, boxHeight, BLUE);//(x point, y point, width (x), height(y), color); 
  tft.setCursor(x_or+10, y_or+10);
  tft.println(buttonTitle);
}

void singleBlueLeftRightButton(int x_or, int y_or, String buttonTitle){ // 
   /* IMPORTANT BUTTON: Option button to choose/increase values. This button has 
      Left and right arrows to increase/decrease parameter values.
    Arg:
      x_or: x location of button
      y_or: y location of button
      buttonTitle: parameter name for button
  */ 
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.fillRect(x_or, y_or, boxWidthMM, boxHeightMM, BLUE);//(x point, y point, width (x), height(y), color); 
  tft.setCursor(x_or+5, y_or+10);
  tft.println(buttonTitle);

  // First row change button
  // LEFT
  tft.setTextColor(YELLOW);
  tft.setTextSize(3);
  tft.fillRect(x_or-sqrButt, y_or, sqrButt, sqrButt, RED); 
  tft.setCursor(x_or-30, y_or+7);
  tft.println("<");
  //RIGHT
  tft.setTextColor(YELLOW);
  tft.setTextSize(3);
  tft.fillRect(240-(x_or-sqrButt)-sqrButt, y_or, sqrButt, sqrButt, RED);
  tft.setCursor(x_or+150, y_or+7);
  tft.println(">");

}

void startButton(){ // 
  /* Indicate: Button is idle
    Arg:
      N/A
  */
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.fillRect(25, 270, boxWidthMM/2, boxHeightMM, RED);//(x point, y point, width (x), height(y), color); 
  tft.setCursor(31, 280);
  tft.println("START");
}

void startButtonENGAGED(){ // 
  /* Indicate: Button is pressed 
    Arg:
      N/A
  */
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.fillRect(25, 270, boxWidthMM/2, boxHeightMM, GREEN);//(x point, y point, width (x), height(y), color); 
  tft.setCursor(31, 280);
  tft.println("START");
}

void stopButton(){ 
  /* Indicate: Button is idle
    Arg:
      N/A
  */
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.fillRect(240-25-boxWidthMM/2, 270, boxWidthMM/2, boxHeightMM, BLUE);//(x point, y point, width (x), height(y), color); 
  tft.setCursor(240-25-boxWidthMM/2+11, 280);
  tft.println("STOP");
}

void stopButtonENGAGED(){ 
  /* Indicate: Button is pressed (for a while)
    Arg:
      N/A
    */
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.fillRect(240-25-boxWidthMM/2, 270, boxWidthMM/2, boxHeightMM, GREEN);//(x point, y point, width (x), height(y), color); 
  tft.setCursor(240-25-boxWidthMM/2+11, 280);
  tft.println("STOP");
}

void redraw(int x_or, int y_or, int mode, String title){ 
  /* Redraws options with left-right arrows
    Arg:
      x_or: x location
      y_or: y location
      mode: parameter number
      title: parameter name
    */
  singleBlueLeftRightButton(x_or, y_or, title + " " + (mode+1));
  delay(20);// end if active
}

String generateActivationCodeString(int x, int y, int z){ //generate 3 code
  /* Generates the activation code of program (for active modes)
    Arg:
      x: first digit
      y: second digit
      z: third digit
    */
  x = x*100;
  y = y*10;
  return String(x+y+z);
}

void authorSign (int x, int y, String codeYear, String authName){
    /* Author signature
    Arg:
      x: x location of string
      y: y location of string
      codeYear: when the code was written
      authName: name of the author
    */
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.setCursor(x, y);
  tft.println("copyright (c) " + codeYear + ", " + authName);
}
