#ifndef _INPUTFUNCTIONS_H_
#define _INPUTFUNCTIONS_H_

#ifndef _CONTROL_SYSTEM_H_
#include "Control_System.h"
#endif

#ifndef _COMMUNICATIONS_H_
#include "Communications.h"
#endif

char receivedChar;
boolean newData = false;
int char2int;

void recvOneChar() 
{
    if (Serial.available() > 0) {
        receivedChar = Serial.read();        
        newData = true;
    }
}



void showNewData() 
{
    if (newData == true) {
        //Serial.print("This just in ... ");
        //Serial.println(receivedChar);
    }
}

void inputTreatment(Control_System* myCS, int hisSlaveAddress, int analogInPin, int analogOutPin, bool* send2RPIFlag)
{
      //if there is new data to treat, treat it
      if(newData==true)
      {
        if(receivedChar=='r')
        {
//          Serial.println("NEW R");//DEBUG
//         
//          delay(1000);
//          *send2RPIFlag=false;
//          delay(1000);
          while(send2RPIFlag==false){delay(1);}
          sendDouble(double(1),hisSlaveAddress,'8');
          *send2RPIFlag=false;
          myCS->taskQ=1;
          
        }
        if(receivedChar=='1')
        {
          //Serial.println("NEW 1");//DEBUG
          //while(Serial.available()<=0){delay(1);}
          receivedChar=Serial.read();
          int receivedInt=receivedChar-'0';
          double suggestedDouble=25*(1+receivedInt);
          
          if(suggestedDouble!=myCS->getLowerBound())
          {
            myCS->changeLowerBound(suggestedDouble,1);
            myCS->taskQ=2;
            
            while(send2RPIFlag==false){delay(1);}
            sendDouble(suggestedDouble, hisSlaveAddress, 'a'); 
            *send2RPIFlag=false;
          }
        }
        if(receivedChar=='2')
        {
          //Serial.println("NEW 2");//DEBUG
          //while(Serial.available()<=0){delay(1);}
          receivedChar=Serial.read();
          int receivedInt=receivedChar-'0';
          double suggestedDouble=25*(1+receivedInt);
          
          if(suggestedDouble!=myCS->getLowerBound(1))
          {
            myCS->changeHisLowerBound(suggestedDouble);
            myCS->taskQ=2;

            while(send2RPIFlag==false){delay(1);}
            sendDouble(suggestedDouble, hisSlaveAddress, 'b');
            *send2RPIFlag=false;
          }
        }
        if(receivedChar=='z')
        {
          
          myCS->distributedControl=!myCS->distributedControl;
          if(myCS->distributedControl==false)
          {
            myCS->changeSetPoint(myCS->getLowerBound());
          }else{
            myCS->FBon=0;
            myCS->initialize=true;
            myCS->taskQ=2;
          }
          
          while(send2RPIFlag==false){delay(1);}
          sendDouble(double(1), hisSlaveAddress, 'c');
          *send2RPIFlag=false;
        }
      }
      newData=false;
}

#endif //_INPUTFUNCTIONS_H_
