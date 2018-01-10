#include <EEPROM.h>

//File with I2C communications functions
#ifndef _COMMUNICATIONS_H_
#include "Communications.h"
#endif

//Read my and the other arduino's slave address (to distinguish)
const int mySlaveAddress = EEPROM.read(0);
const int hisSlaveAddress = EEPROM.read(1);

double delayValue=1;
int count;

void setup() {
  Serial.begin(230400);
  beginComs( mySlaveAddress);
}

void loop() {
  
  if((mySlaveAddress==10)&&(delayValue<10000))
  {
      for(count=0;count<5;count++)
      {
        Wire.beginTransmission(hisSlaveAddress);
        Wire.write('a');
        Wire.endTransmission();
        delayMicroseconds(delayValue);
      }
      sendDouble(delayValue,hisSlaveAddress, '1');
      delayValue+=5;
  }

}
