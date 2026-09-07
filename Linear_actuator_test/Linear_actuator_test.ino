#include "TeensyStep.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce.h>

Encoder knob(11, 12);
const int encoder_SW = 13;
Bounce pushbutton = Bounce(encoder_SW, 50);  // 10 ms debounce
bool state = 0;

const int Enable = 2;
const int Enable_2 = 5;

Stepper My(6, 7);       // (STEP pin, DIR pin)  // The stepper class encapsulates the physical properties of a stepper motor like pin numbers of the STEP and DIR signals, speed and acceleration of the motor.
Stepper Mx(3, 4);
 
StepControl step_controller;    // The StepControl class is used to synchronously move up to 10 motors to their target positions.

float x;
float y;
const float Y0 = 67;
const float X0 = 15;
byte Xmax = 5;
byte Ymax = 3;
const float delta = 18.9;

bool dirx = true;
bool diry = true;
float f;
float t;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MS1 = 8;
const int MS2 = 9;
const int MS3 = 10;

const int SWx = 14;
const int SWy = 15;

float um_per_step = 1.25; //400 steps/second
int steps = 0;
int steps_rel;

void setup()
{
  pinMode(MS1, OUTPUT);    // set the MS1, MS2, MS3 and Enable as an outputs
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  
  pinMode(Enable, OUTPUT);
  pinMode(Enable_2, OUTPUT);


  digitalWrite(MS1, HIGH); 
  digitalWrite(MS2, LOW); 
  digitalWrite(MS3, LOW);
  
  digitalWrite(Enable, LOW); // set the enables to LOW
  digitalWrite(Enable_2, LOW); 


  pinMode(SWx, INPUT_PULLUP);
  pinMode(SWy, INPUT_PULLUP);
  
  // Set the motor max acceleration
  My.setMaxSpeed(10000)       // steps/s
      .setAcceleration(20000)   // stp/s^2
      .setInverseRotation(true);

  Mx.setMaxSpeed(10000)       // steps/s
      .setAcceleration(20000)   // stp/s^2
      .setInverseRotation(true);
      
  pinMode(encoder_SW, INPUT_PULLUP);
     
  Serial.begin(115200);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(1000); 
  display.clearDisplay();

  OLED_display();
  
  delay(2000); 
  
    // Homing routine

//  while (digitalRead(SWx) == HIGH)      
//  {
//    TurnClockwise_M1 (1000);
//    Serial.println(digitalRead(SWx));
//  }
//  while (digitalRead(SWy) == HIGH)      
//  {
//    TurnClockwise_M2 (100);
//    Serial.println(digitalRead(SWy));
//  }
}

void loop() 
{
    OLED_display();
    //read_pushbutton();
    //steps = knob.read()/2;
    steps = steps - 100;
    //Serial.println(steps); 
    Mx.setTargetAbs(steps);
    step_controller.move(Mx); 
    delay(3000); 

    
//    if (pushbutton.update()) 
//    {
//      if (pushbutton.fallingEdge()) 
//      {
//        knob.write(0);
//      }
//    } 
   
       
//       delay(t*1000);
//       Mx.setTargetAbs(0);
//       step_controller.move(Mx);
//       delay(t*1000);
  
}

// Function to get the steps from the mm
int mm_to_step (float mm)
{
  int steps;
  steps = mm*1600;// 1600 steps per mm
  return steps;
}

// Function to get the mm from the steps
float step_to_mm (int steps)
{
  float mm;
  mm = steps*0.000625; // 6.25 e-4 mm per step
  return mm;
}

//Function to rotate the left motor CW
void TurnClockwise_M1 (int pd)
{
   digitalWrite(4,1);
   digitalWrite(3,HIGH);
   digitalWrite(3,LOW);
   delayMicroseconds(pd);
}

//Function to rotate the right motor CW
void TurnClockwise_M2 (int pd)
{
   digitalWrite(7,1);
   digitalWrite(6,HIGH);
   digitalWrite(6,LOW);
   delayMicroseconds(pd);
}

void read_pushbutton()
{
    if (pushbutton.update()) 
    {
      if (pushbutton.fallingEdge()) 
      {
        state = !state;
      }
    } 
}

void OLED_display()
{   
    display.clearDisplay();
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("Test"));
    display.println();
    display.print(F("Distance:"));
    display.print(-1*steps*um_per_step);
    display.println(F(" um"));
    display.display(); 
}
