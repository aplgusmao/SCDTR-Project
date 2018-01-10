#ifndef _COMMUNICATIONS_H_

#include "Communications.h"
#endif


//Communications file has access to our control system object to update it's values based on received information

//Communications initialization, 
//mySlaveAddress is the address with which the arduino is registered on the Bus, 
//auxCS is a pointer to our control system object
void beginComs(int mySlaveAddress)
{
  Wire.begin(mySlaveAddress);//Initialize communications, register with address stored in EEPROM
  Wire.onReceive(receiveEvent);//set receiveEvent as function to run on receive.
}

//send a double instead of a single byte.
//message is the double to be sent
//hisSlaveAddress is the address to which the double should be sent
//flag will tell the other Arduino what to do with the information
void sendDouble(double message, int hisSlaveAddress, byte flag)
{  
  
  //Unions allow one portion of memory to be accessed as different data types,all its member elements occupy the same physical space in memory
  union {
    double double_variable;
    byte temp_array[8];
  } u;
  u.double_variable=message; //u.temp_array is now the message's conversion to an 8 byte array.

  
  byte comsBus[9];

  //build the bus of bytes to be sent
  for(int i=0; i<8;i++){comsBus[i]=u.temp_array[i];}
  comsBus[8]=flag;

  //DEBUG
  //Serial.print("Message sent: ");
  //Serial.print(message);
  //Serial.print("\t with flag:");
  //Serial.println((char)flag);

  //send byte bus to arduino registered with hisSlaveAddress
  Wire.beginTransmission(hisSlaveAddress);
  Wire.write(comsBus, 9);
  Wire.endTransmission();
}


//Function to be run on the event of incoming information
//howMany indicates the size of the message in bytes
void receiveEvent(int howMany)
{
  char aux,x, flag;
  byte byteMessage[8];

  if(howMany!=9)
  {
    while(Wire.available()>0)
    {
      x=Wire.read();
      Serial.print(x);
    }
    Serial.println('e');
  }else{
    union {
      double double_variable;
      byte temp_array[8];
    } u;
   //u.temp_array is now the message's conversion to an 8 byte array.
  
    
    //for(int i=0;i<8;i++){byteMessage[i]=Wire.read();}
  
    //the first 8 bytes represent the message and the last the flag
    for(int i=0;i<8;i++){u.temp_array[i]=Wire.read();} //u.double_variable is now the message's conversion to a double.
    flag=Wire.read();
  
    double doubleMessage=u.double_variable;
    
    //DEBUG
    //Serial.print("Message received: ");
    //Serial.print(doubleMessage);
    //Serial.print("\t with flag:");
    //Serial.println(flag);
  
    //the flag indicates what needs to be done
    switch(flag)
    {
       case '1':
          Serial.println(doubleMessage);
    }
  }
  
  
  
}

