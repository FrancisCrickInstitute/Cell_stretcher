#include "TeensyStep.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Encoder knob(26, 27);
Encoder knob_2(23, 24);
Encoder knob_3(20, 21);
Encoder knob_4(15, 16);

const int encoder_SW = 25;
const int encoder_SW_2 = 22;
const int encoder_SW_3 = 17;
const int encoder_SW_4 = 14;

const int Enable = 2;
const int Enable_2 = 5;
const int Enable_3 = 8;
const int Enable_4 = 11;

Stepper M1(3, 4);       // STEP pin: 7, DIR pin: 6  // The stepper class encapsulates the physical properties of a stepper motor like pin numbers of the STEP and DIR signals, speed and acceleration of the motor.
Stepper M2(6, 7);
Stepper M3(9, 10);
Stepper M4(12, 13);

const int SW1 = 36;
const int SW2 = 35;
const int SW3 = 38;
const int SW4 = 37;

StepControl step_controller;    // The StepControl class is used to synchronously move up to 10 motors to their target positions.

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MS1 = 28;
const int MS2 = 29;
const int MS3 = 30;

float um_per_step = 1.6; //400 steps/second
int steps = 0;

void setup()
{
  pinMode(MS1, OUTPUT);    // set the MS1, MS2, MS3 and Enable as an outputs
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  pinMode(Enable, OUTPUT);
  pinMode(Enable_2, OUTPUT);
  pinMode(Enable_3, OUTPUT);
  pinMode(Enable_4, OUTPUT);

  digitalWrite(MS1, HIGH); // sets the digital pin MS1, MS2, MS3 to HIGH
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

  digitalWrite(Enable, LOW); // set the enables to LOW
  digitalWrite(Enable_2, LOW);
  digitalWrite(Enable_3, LOW);
  digitalWrite(Enable_4, LOW);

  // Set the motor max acceleration
  M1.setMaxSpeed(5000)
  .setAcceleration(10000)   // stp/s^2
  .setInverseRotation(true);

  M2.setMaxSpeed(5000)
  .setAcceleration(10000)   // stp/s^2
  .setInverseRotation(true);

  M3.setMaxSpeed(5000)
  .setAcceleration(10000)   // stp/s^2
  .setInverseRotation(true);

  M4.setMaxSpeed(5000)
  .setAcceleration(10000)   // stp/s^2
  .setInverseRotation(true);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
  
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  delay(1000);
  display.clearDisplay();

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

void loop()
{
  steps = 3000;
  M1.setTargetAbs(steps);
  M2.setTargetAbs(steps);
  M3.setTargetAbs(steps);
  M4.setTargetAbs(steps);
  step_controller.move(M1, M2, M3, M4);
  delay(500);
  
  steps = 2500;
  M1.setTargetAbs(steps);
  M2.setTargetAbs(steps);
  M3.setTargetAbs(steps);
  M4.setTargetAbs(steps);
  step_controller.move(M1, M2, M3, M4);
  delay(500);
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
  digitalWrite(4, 1);
  digitalWrite(3, HIGH);
  digitalWrite(3, LOW);
  delayMicroseconds(pd);
}

//Function to rotate the right motor CW
void TurnClockwise_M2 (int pd)
{
  digitalWrite(7, 1);
  digitalWrite(6, HIGH);
  digitalWrite(6, LOW);
  delayMicroseconds(pd);
}

void TurnClockwise_M3 (int pd)
{
  digitalWrite(10, 1);
  digitalWrite(9, HIGH);
  digitalWrite(9, LOW);
  delayMicroseconds(pd);
}

void TurnClockwise_M4 (int pd)
{
  digitalWrite(13, 1);
  digitalWrite(12, HIGH);
  digitalWrite(12, LOW);
  delayMicroseconds(pd);
}


//void read_pushbutton()
//{
//  if (pushbutton.update())
//  {
//    if (pushbutton.fallingEdge())
//    {
//      state = !state;
//    }
//  }
//}

void OLED_display()
{
  display.clearDisplay();
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println(F("Test"));
  display.println();
  display.print(F("Distance:"));
  display.print(-1 * steps * um_per_step);
  display.println(F(" um"));
  display.display();
}
