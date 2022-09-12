#include "TeensyStep.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce.h>

//Encoder freq_knob(37, 36);//Change 38,37 with new PCB
//Encoder stretch_knob(35, 34);
Encoder stretch_knob(37, 36);

const int encoder_SW = 39;
//const int encoder_SW_2 = 36;//Add with new PCB

// Instantiate a Bounce object with a 5 millisecond debounce time
Bounce pushbutton = Bounce( encoder_SW,20 ); 
//Bounce pushbutton2 = Bounce( encoder_SW_2,20 ); 

const int Enable = 1;

Stepper M1(7, 8);       // STEP pin: 7, DIR pin: 6  // The stepper class encapsulates the physical properties of a stepper motor like pin numbers of the STEP and DIR signals, speed and acceleration of the motor.
Stepper M2(11, 12);
Stepper M3(26, 27);
Stepper M4(30, 31);

const int SW1 = 15;
const int SW2 = 17;
const int SW3 = 21;
const int SW4 = 23;

StepControl step_controller;    // The StepControl class is used to synchronously move up to 10 motors to their target positions.

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MS1 = 2;
const int MS2 = 3;
const int MS3 = 4;

float um_per_step = 1.6; //400 steps/second
int steps = 0;
const int initial_steps = 3000;

float freq = 0;
float freq_factor = 0.05;
float stretch = 0;
float T;
const float initial_stretch_diameter = 14.6;
float p_stretch;
unsigned long t = 0;
bool state = 0;

void setup()
{
  pinMode(MS1, OUTPUT);    // set the MS1, MS2, MS3 and Enable as an outputs
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  pinMode(Enable, OUTPUT);

  digitalWrite(MS1, HIGH); // sets the digital pin MS1, MS2, MS3 to HIGH
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  digitalWrite(Enable, LOW); // set the enables to LOW

  // Set the motor max acceleration
  M1.setMaxSpeed(1000)
  .setAcceleration(5000)   // stp/s^2
  .setInverseRotation(true);

  M2.setMaxSpeed(1000)
  .setAcceleration(5000)   // stp/s^2
  .setInverseRotation(true);

  M3.setMaxSpeed(1000)
  .setAcceleration(5000)   // stp/s^2
  .setInverseRotation(true);

  M4.setMaxSpeed(1000)
  .setAcceleration(5000)   // stp/s^2
  .setInverseRotation(true);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
  
  pinMode(encoder_SW, INPUT_PULLUP);
  //pinMode(encoder_SW_2, INPUT_PULLUP);
  
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  delay(1000);
  display.clearDisplay(); 

  Homing_routine();

  // Go to initial position
  M1.setTargetAbs(initial_steps);
  M2.setTargetAbs(initial_steps);
  M3.setTargetAbs(initial_steps);
  M4.setTargetAbs(initial_steps);
  step_controller.move(M1, M2, M3, M4);
}

void loop()
{

  //freq = abs(freq_knob.read())/4*freq_factor; // Select the frequency
  freq = 1; // Select the frequency
  stretch = abs(stretch_knob.read())/4*0.05; // select the stretch
  p_stretch = 2*stretch/(initial_stretch_diameter)*100; //2 times because it is in 2 directions simultaneously
  T = 1/freq; // Calculate the period
  t = T*1000; // Convert the period in milliseconds
  OLED_display(); // Display selection

  // Get steps from mm selected by the user using the stretch knob
  steps = mm_to_step(stretch/2); // Divided by two beacuse it is what will move each stretcher

  read_pushbutton();

  if (state == 1)
  {
    read_pushbutton();
    
    // Set the target for each motor and move to the target
    M1.setTargetAbs(initial_steps-steps);
    M2.setTargetAbs(initial_steps-steps);
    M3.setTargetAbs(initial_steps-steps);
    M4.setTargetAbs(initial_steps-steps);
    step_controller.move(M1, M2, M3, M4);

    delay(t);

    // Com back to the origin
    M1.setTargetAbs(initial_steps);
    M2.setTargetAbs(initial_steps);
    M3.setTargetAbs(initial_steps);
    M4.setTargetAbs(initial_steps);
    step_controller.move(M1, M2, M3, M4);
  }

}

// Function to get the steps from the mm
int mm_to_step (float mm)
{
  int steps;
  steps = mm * 1600; // 1600 steps per mm
  return steps;
}

// Function to get the mm from the steps
float step_to_mm (int steps)
{
  float mm;
  mm = steps * 0.001600; // 1.587 um per step
  return mm;
}

//Function to rotate the left motor CW
void TurnClockwise_M1 (int pd)
{
  digitalWrite(8, 1);
  digitalWrite(7, HIGH);
  digitalWrite(7, LOW);
  delayMicroseconds(pd);
}

//Function to rotate the right motor CW
void TurnClockwise_M2 (int pd)
{
  digitalWrite(12, 1);
  digitalWrite(11, HIGH);
  digitalWrite(11, LOW);
  delayMicroseconds(pd);
}

void TurnClockwise_M3 (int pd)
{
  digitalWrite(27, 1);
  digitalWrite(26, HIGH);
  digitalWrite(26, LOW);
  delayMicroseconds(pd);
}

void TurnClockwise_M4 (int pd)
{
  digitalWrite(31, 1);
  digitalWrite(30, HIGH);
  digitalWrite(30, LOW);
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

void Homing_routine()
{
  
  while (digitalRead(SW1) == HIGH)
  {
    TurnClockwise_M1 (1000);
    Serial.println(digitalRead(SW1));
  }

  while (digitalRead(SW2) == HIGH)
  {
    TurnClockwise_M2 (1000);
    Serial.println(digitalRead(SW2));
  }

  while (digitalRead(SW3) == HIGH)
  {
    TurnClockwise_M3 (1000);
    Serial.println(digitalRead(SW3));
  }

  while (digitalRead(SW4) == HIGH)
  {
    TurnClockwise_M4 (1000);
    Serial.println(digitalRead(SW4));
  }
}
void OLED_display()
{
  display.clearDisplay();
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println(F("8 DOF cell stretcher"));
  display.println();
  display.print(F("Frequency:"));
  display.print(freq);
  display.println(F(" Hz"));
  //display.println();
  display.print(F("Stretch:"));
  display.print(stretch);
  display.println(F(" mm"));
  //display.println();
  display.print(F("Steps stretch:"));
  display.println(steps);
  //display.println();
  display.print(F("Period:"));
  display.print(T);
  display.println(F(" s"));
  //display.println();
  display.print(F("%Stretch:"));
  display.print(p_stretch);
  display.println(F(" %"));
  display.print(F("State:"));
  //display.print(state);
  if (state == 0){display.print(F("Inactive"));}
  if (state == 1){display.print(F("Active"));}
  display.display();
}
