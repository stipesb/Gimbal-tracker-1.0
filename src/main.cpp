#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// za adafruit senzor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  // adafruit*

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);   // konstruktor za adafruit
                // adafruit bno055 communicate with i2c communication, that are
                // connected on A4 & A5 pins of arduino


 int Kp = 4;    // proportional parameter of controller


float *x_pocetni;
float *y_pocetni;

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 2000
// Target RPM for X axis motor
#define MOTOR_X_RPM 14
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 14

// X motor pins
#define DIR_X 5
#define STEP_X 6


// Y motor pins
#define DIR_Y 7
#define STEP_Y 8

// Z motor

//#define DIR_Z 9
//#define STEP_Z 10

//modes of stepping MS pins
#define MS1 2
#define MS2 3
#define MS3 4


// #define pin_lijevo A6
// #define pin_drugi_livo A7



BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);

SyncDriver controller(stepperX, stepperY);

void setup() {

for(int i = 2; i<11; i++) // define pins as output
{
  pinMode(i,OUTPUT); 
}

// MS1 MS2 MS3                 // stepping modes table
//  0   0   0   1/1
//  1   0   0   1/2
//  0   1   0   1/4
//  1   1   0   1/8
//  1   1   1   1/16
#define MICROSTEPS 16    
digitalWrite(MS1,HIGH);
digitalWrite(MS2,HIGH);
digitalWrite(MS3,HIGH);

    stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
 

// setup for adafruit sensor_________________________________________________

 // Serial.println("Orientation Sensor Test"); Serial.println("");
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  sensors_event_t event; 
  bno.getEvent(&event);

float x_test = event.orientation.x;
float y_test = event.orientation.y;
x_pocetni = &x_test;
y_pocetni = &y_test;

 
//__________________________________________________________________________

}

  sensors_event_t event; 


void loop() {
int i = 1;
int k = 1;


  bno.getEvent(&event);


  float x_trenutni = event.orientation.x;
  float y_trenutni = event.orientation.y;


if(x_trenutni>180)
{
x_trenutni = x_trenutni -360;
}

int zakreni_y_za =*y_pocetni-y_trenutni;
int zakreni_x_za =x_trenutni - *x_pocetni ;

if(zakreni_x_za < 0)
{
k = -1;

zakreni_x_za = - zakreni_x_za;

}
else if(zakreni_x_za == 0)
{
  k = 0;
}

if(zakreni_y_za<0)
{
i = -1;

zakreni_y_za =0 - zakreni_y_za;

}
else if(zakreni_y_za == 0)
{
  i = 0;
}


stepperY.setRPM(zakreni_y_za);


stepperX.setRPM(Kp * zakreni_x_za);


controller.rotate(k,0);


}