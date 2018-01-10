#include <EEPROM.h> //required to read address for EEPROM and distinguish ARDUINOS  
#include <Wire.h> //required for I2C communication

//This file contains the functions for receiving and treating information through serial port
#ifndef _INPUTFUNCTIONS_H_
#include "inputFunctions.h"
#endif


#include <math.h>

//this file describes our countrol system (PID+FF)
#ifndef _CONTROL_SYSTEM_H_
#include "Control_System.h"
#endif

//File with I2C communications functions
#ifndef _COMMUNICATIONS_H_
#include "Communications.h"
#endif

const int analogInPin = A0; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
int sensorValue = 0; // value read from the pot
double outputValue = 0; // value output to the PWM (analog out)
double sensorValueLux = 0; // value read from the pot in LUX

unsigned long sampleRate=20; //in miliseconds
unsigned long startTime=0;
unsigned long endTime=0;

//PID parameters, D is 0 but aPID has to be != 0
double Kp=3;//3;//5//10;
double Ki=1;//1;//3
double Kd=0.0;
double aPID=1;
double bPID=1;

//Read my and the other arduino's slave address (to distinguish)
const int mySlaveAddress = EEPROM.read(0);
const int hisSlaveAddress = EEPROM.read(1);
const int RPIaddress=0x48;


//initial setpoint, equal to both arduinos
double initialSetpoint=25;

//flag that indicates it's okay to start next iteration
bool* send2RPI;


//Build a control system object, set PID parameters and identification (hisSlaveAddress)
Control_System myCS(&sensorValueLux,&outputValue,initialSetpoint,Kp,Ki,Kd,sampleRate,bPID,aPID, hisSlaveAddress); 

int count=0;

void setup() {
  Serial.begin(230400);
  beginComs(mySlaveAddress, &myCS, &send2RPI);//Communications file has access to our control system object to update it's values based on received information
  
  delay(3000); //wait just enought time to close box lid
  myCS.calibration(analogInPin,analogOutPin,hisSlaveAddress); //Calibrate CS, get coupling factors and illumination offsets


  myCS.consensus(); //run consensus, find optimal setpoint and FF pwm
  double cost=myCS.systemCostFunction2( );
  Serial.print("Cooperative Solution:");
  Serial.println(cost);

  
  myCS.dConsensus[0]=(initialSetpoint-myCS.illuminationOffset[0])/myCS.couplings[0][0];
  myCS.dConsensus[1]=(initialSetpoint-myCS.illuminationOffset[1])/myCS.couplings[1][1];
  cost=myCS.systemCostFunction2( );
  Serial.print("Noncooperative Solution:");
  Serial.println(cost);
  
}

void loop() {

}

