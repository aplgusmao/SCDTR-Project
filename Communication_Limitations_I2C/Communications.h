#ifndef _COMMUNICATIONS_H_
#define _COMMUNICATIONS_H_

#ifndef _ARDUINO_H_
#include "Arduino.h"
#endif

#ifndef _WIRE_H_
#include <Wire.h>
#endif

//definition of a byte
#define byte uint8_t

void beginComs(int address);
void sendDouble(double message, int hisSlaveAddress, byte flag);
void receiveEvent(int howMany);


#endif //_COMMUNICATIONS_H_
