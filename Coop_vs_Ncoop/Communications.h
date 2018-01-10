#ifndef _COMMUNICATIONS_H_
#define _COMMUNICATIONS_H_

#ifndef _ARDUINO_H_
#include "Arduino.h"
#endif

#ifndef _WIRE_H_
#include <Wire.h>
#endif

#ifndef _CONTROL_SYSTEM_H_
#include "Control_System.h"
#endif

//definition of a byte
#define byte uint8_t

//forward declaration of type Control_System
class Control_System;

void beginComs(int address, Control_System* auxCS, bool** send2R);
void sendDouble(double message, int hisSlaveAddress, byte flag);
void receiveEvent(int howMany);
void sendData2RPI(int RPIaddress, int hisSlaveAddress);


#endif //_COMMUNICATIONS_H_
