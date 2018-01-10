#ifndef _COMMUNICATIONS_H_

#include "Communications.h"
#endif


//Communications file has access to our control system object to update it's values based on received information
Control_System* comsCS;
bool send2RPIFlag;
char ID='2';
bool start=true;


//Communications initialization, 
//mySlaveAddress is the address with which the arduino is registered on the Bus, 
//auxCS is a pointer to our control system object
void beginComs(int mySlaveAddress, Control_System* auxCS, bool** send2R)
{
  Wire.begin(mySlaveAddress);//Initialize communications, register with address stored in EEPROM
  Wire.onReceive(receiveEvent);//set receiveEvent as function to run on receive.
  comsCS=auxCS;
  *send2R=&send2RPIFlag;
  
  if(mySlaveAddress==10){ID='1';}
  else{ID='2';}
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
        //DEBUG
        //Serial.println("EXECUTING FLAG 1"); 
        //this indicates the other Arduino's LED is ON and has calculated his couplings[0][0], so we can calculate couplings[0][1]
        comsCS->changeCouplings( 1, 1, doubleMessage);
        break;
     case '2':
        //this indicates the other Arduino is sending me information regarding his couplings[0][1] and mine couplings[1][0]
        comsCS->changeCouplings( 1, 0, doubleMessage);
        break;
     case '3':
        //this indicates the other Arduino is sending me information regarding his illuminationOffset[0] and mine illuminationOffset[1]
        comsCS->changeOffset(1, doubleMessage);
        break;
     case '4':
        //the other Arduino is sending me it's consensus iteration solution (first half)
        comsCS->changeCopy(0, doubleMessage);
        break;
     case '5':
        //the other Arduino is sending me it's consensus iteration solution (second half)
        comsCS->changeCopy(1, doubleMessage);
        comsCS->nextIteration=1;
        break;
     case '6':
        //the other Arduino has changed his occupancy, it's necessary to run another consensus
        comsCS->changeHisLowerBound(doubleMessage);
        comsCS->consensus();
        break;
     case '7':
        //the other Arduino is telling me the bus is free and it's okay to send data to RPI
        send2RPIFlag=true;
        break;
     case '8':
        //Queue reset
        //Serial.println("QUEUEING");
        comsCS->taskQ=1;
        send2RPIFlag=true;
        break;
     case '9':
        //the other is ready to reset
        comsCS->resetReady=true;
        break;
     case 'a':
        //Update his lower bound, queue consensus
        comsCS->changeHisLowerBound(doubleMessage);
        comsCS->taskQ=2;
        send2RPIFlag=true;
        break;
     case 'b':
        //Update his lower bound, queue consensus
        comsCS->changeLowerBound(doubleMessage,1);
        comsCS->taskQ=2;
        send2RPIFlag=true;
        break;
     case 'c':
        //toggle distributed control
        comsCS->distributedControl=!comsCS->distributedControl;
        if(comsCS->distributedControl==false)
        {
          comsCS->changeSetPoint(comsCS->getLowerBound());
        }else{
            comsCS->FBon=0;
            comsCS->initialize=true;
            comsCS->taskQ=2;
        }
        send2RPIFlag=true;
        break;
     case 'd':
        Serial.print("\t2- \t");
        Serial.println(doubleMessage);
  }
  
}

void sendDouble2RPI(double doubleMessage, int RPIaddress)
{
  String messageString = String(doubleMessage,3);
  char messageChar[messageString.length()];
  messageString.toCharArray(messageChar,messageString.length());

   Wire.beginTransmission(RPIaddress);
   Wire.write(messageChar);
   Wire.endTransmission();
}

void sendData2RPI(int hisSlaveAddress, int RPIaddress)
{ 
  char occupancy;
  if(hisSlaveAddress==11 && start==true){send2RPIFlag=true;start=false;ID='1';}//Arduino #10 starts
  
  double dutyCycle=double(round(*(comsCS->output))/255.0);
  double setP=comsCS->getSetpoint();
  double illumination=*(comsCS->input);

  String dutyString = String(dutyCycle,3);
  String setPString = String(setP,3);
  String illString = String(illumination,3);

  char dutyChar[dutyString.length()];
  int dutySize=sizeof(dutyChar);
  dutyString.toCharArray(dutyChar,dutyString.length());
//  Serial.println(dutyChar);
  
  char setPChar[setPString.length()];
  int setPSize=sizeof(setPChar);   
  setPString.toCharArray(setPChar,setPString.length());
//  Serial.println(setPChar);
  
  char illChar[illString.length()];
  int illSize=sizeof(illChar); 
  illString.toCharArray(illChar,illString.length());
  //DEBUG
//  Serial.println(illChar);
 
  
  char auxMessage[16];
  for(int i=0;i<16;i++){auxMessage[i]='f';}
  //DEBUG
//  Serial.println(auxMessage);
  auxMessage[0]=ID;
  //DEBUG
//  Serial.println(dutySize);
  //DEBUG
//  Serial.println(dutyChar);
  for(int i=0;i<dutySize-1;i++){auxMessage[i+1]=dutyChar[i];}
  //DEBUG
//  Serial.println(auxMessage);
  for(int i=0;i<setPSize-1;i++){auxMessage[i+dutySize]=setPChar[i];}
  //DEBUG
//  Serial.println(auxMessage);
  for(int i=0;i<illSize-1;i++){auxMessage[i+dutySize+setPSize-1]=illChar[i];}

  if(comsCS->getLowerBound()==50){occupancy='1';}
  else{occupancy='0';}

  auxMessage[dutySize+setPSize+illSize-2]=occupancy;
  
  //DEBUG
  //Serial.println(auxMessage);
  
  while(send2RPIFlag==false){delay(100);}
  Wire.beginTransmission(RPIaddress);
  Wire.write(auxMessage);
  Wire.endTransmission();

  sendDouble(double(1), hisSlaveAddress, '7');
  send2RPIFlag=false;
}

