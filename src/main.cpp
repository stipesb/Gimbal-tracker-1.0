#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"


// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 2000
// Target RPM for X axis motor
#define MOTOR_X_RPM 7
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 7

// X motor
#define DIR_X 2
#define STEP_X 3


// Y motor
#define DIR_Y 11
#define STEP_Y 12


#define MICROSTEPS 16

BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);

SyncDriver controller(stepperX, stepperY);

void setup() {
  Serial.begin(9600);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
pinMode(9,OUTPUT);

digitalWrite(7,HIGH);

digitalWrite(8,HIGH);
digitalWrite(9,HIGH);

    stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
 

    Serial.setTimeout(5);


}

void loop() {

String a = Serial.readString();

if(a=="w")
{
  controller.rotate(-30, 0);

}

if(a=="a")
{
 controller.rotate(0, 30);

  
}
if(a=="s")
{
 controller.rotate(30, 0);

  
}
if(a=="d")
{
 controller.rotate(0, -30);

  
}

  if(a=="t")
{
 controller.rotate(35, -30);

   controller.rotate(15, 20);
     controller.rotate(-15, -20);
   controller.rotate(-35, 30);   
}

}