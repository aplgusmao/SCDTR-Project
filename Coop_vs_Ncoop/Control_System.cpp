#ifndef _CONTROL_SYSTEM_H_
#include "Control_System.h"
#endif


//constructor function
// Input, output are pointers since it's impratical to update these values in each PID loop 
// HSA contains the other arduino's slave address, this will be used for I2C communications
Control_System::Control_System(double *inp, double *outp, double setp, double Kp1, double Ki1, double Kd1, unsigned long sampleR, double b1, double a1, int HSA): input(inp), output(outp),errorPrev(setp), Kp(Kp1), Ki(Ki1), Kd(Kd1), sampleRate(sampleR>0?sampleR:1),b(b1),a(a1),hisSlaveAddress(HSA)
{
  
  Dprev=0.0;
  //the setpoint is the same for both arduinos
  changeLowerBound(setp, 1);
  L[1]=setp;
  
  //DEBUG 
  //Serial.println("PID object created");
}

//destructor function
Control_System::~Control_System()
{
  //DEBUG
  //Serial.println("PID object destroyed");  
}


//compute system's response to current state
void Control_System::compute( )
{
  //auxiliar vector for intermediary calculations
  double aux[11];

  //compute error
  error=(setpoint)-(*input);

  //DEBUG
  //Serial.print("error:");
       //Serial.println(error);
       //Serial.print("FFoff:");
       //Serial.println(FFoff);
       //Serial.print("FBon:");
       //Serial.println(FBon);
  computeTimeStamp=micros();
  if(computeTimeStamp-initializeTimeStamp>20000){FBon=1;}
       
  //compute FF+FB response once error to setpoint is less than 5 LUX (this simulates the system's delay)
  //or compute FB only response if FF is off
  //once in static regime, FB should always be on to account for external responses even if error>5 LUX
  //if the system is in the deadzone, a new response shouldn't be computed
  //the deadzone should be ignored if the PID requires initialization (this happens everytime the system reaches static regime)
  bool auxA=1;
  if( ( (abs(error)<=5 || FFoff==1 || FBon==1 ) && (!checkDeadZone() || initialize)) && auxA)
  {
    //Initialize PID
    if(initialize)
    {
      inputPrev = 0;
      Iprev = 0;
      initialize = false;
      FBon=1;//from this moment on the system is in static regime, FB is on
    }

    //compute proportional value
    P=Kp*(b*(setpoint)-(*input));
    
    //compute integral value
    I=Iprev+Kp*Ki*double(sampleRate/1000)*((error+errorPrev)/2.0);

    //compute differential value
    aux[6]=double(Kd*Dprev);
    aux[7]=double(Kd+a*sampleRate); //if a && Kd==0, division by 0 would occur, so a!=0
    aux[8]=double(aux[6]/aux[7]);
    aux[10]=double(((*input)-inputPrev)*Kp*Kd*a);
    aux[11]=double(aux[10]/aux[7]);
    D=double(aux[8]-aux[11]);

    //if FF is off compute FB only response
    if (FFoff==0) generateFFOutput(setpoint);
    else outputFF=0;

    //generate output
    *output=P+I+D+outputFF;

    
    //Anti-windup 
//    if (*output>255)
//    {
//      I-=((*output)-255);
//      *output=255;
//    }
//    if (*output<0)
//    {
//      I-=(*output);
//      *output=0;
//    }

    
    //resgister values for future computation 
    Iprev=I;
    errorPrev=error;
    inputPrev=(*input);
    Dprev=D;


    
  }else
  {
     //is still in transient regime
     //compute FF only response

     
     if(checkDeadZone())
     {
       //if the system is in the deadzone, use previous response 
       generateFFOutput(setpoint);
       *output=outputFF+P+I+D;
       
       
       //DEBUG
       //Serial.println("in the deadzone");
       //Serial.print("P:");
       //Serial.println(P);
       //Serial.print("I:");
       //Serial.println(I);
       //Serial.print("D:");
       //Serial.println(D);
     }
     else
     {
      //compute FF only response
      generateFFOutput(setpoint);
     *output=outputFF;
     }
     
     //DEBUG
     //Serial.println("THIS IS FF");
  }
  
}

//MAYBE
//change PID parameters
void Control_System::changeParameters(double Kp1, double Ki1, double Kd1, double b1, double a1)
{
  Kp=Kp1;
  Ki=Ki1;
  Kd=Kd1;
  b=b1;
  a=a1;

  //DEBUG
  //Serial.print("PID parameters changed to: Kp-");
  //Serial.print(Kp);
  //Serial.print("\t Ki-");
  //Serial.print(Ki);
  ////Serial.print("\t Kd-");
  //Serial.print(Kd);
  //Serial.print("\t b-");
  //Serial.print(b);
  //Serial.print("\t a-");
  //Serial.print(a);
}

//MAYBE
//change sample rate
void Control_System::changeSampleRate(double sampleR)
{
  sampleRate=(sampleR>0?sampleR:sampleRate); //check if new sample rate>0

  //DEBUG
  //Serial.print("PID sample rate changed to: ");
  //Serial.println(sampleRate);
}

//Change system's setpoint, initFlag has 0 as default value
void Control_System::changeLowerBound(double setp, bool initFlag)
{
  L[0]=setp;
  
  //turn off FB
  FBon=0;
  
  //FB needs to be reinitialized
  initialize = true;

  initializeTimeStamp=micros();

  //if it is not the system's initialization, send the other arduino my new setpoint, run new consensus
  if (initFlag==0){sendDouble(L[0], hisSlaveAddress, '6');consensus();}
}

//Change the element of the couplings matrix with coordinates (line,column) to value  
void Control_System::changeCouplings(int line, int column, double value)
{
  couplings[line][column]=(value>0?value:couplings[line][column]); //check if new value is >0
}

//Change element of the offset array with coordinate index to value
void Control_System::changeOffset(int index, double value)
{
  illuminationOffset[index]=(value>0?value:illuminationOffset[index]); //check if new value is >0
}

//Update the local copy of the other node's previous iteration solution 
void Control_System::changeCopy(int index, double value)
{
  d_his[index]=((value>0)&&(value<100)?value:d_his[index]); //check if 100>value>0
}

//Update the local copy of the other node's setpoint
void Control_System::changeHisLowerBound(double value)
{
  L[1]=(value>0?value:L[1]); //check if new value is >0
  
  //turn off FB
  FBon=0;
  
  //FB needs to be reinitialized
  initialize = true;

  initializeTimeStamp=micros();
}

//Determine Parameters for linear regression in FF output generation
void Control_System::calibration (int analogInPin, int analogOutPin,int hisSlaveAddress)
{
  double point12,point11;
  double percent=25000/255;

  
  

  //arduino with address 10 starts
  if (hisSlaveAddress==11)
  {
    //switch off LED 
    analogWrite(analogOutPin, 0);
    delay(100); // wait for stability
    
    illuminationOffset[0] = convertToLux(analogRead(analogInPin)); //calculate offset
    
    sendDouble(illuminationOffset[0],hisSlaveAddress,'3'); //send my offset to other node
    if(taskQ==1){sendDouble(illuminationOffset[0], hisSlaveAddress, '8');}
    
    while(illuminationOffset[1]==0){delay(100);} //wait for the other node to calculate his offset before turning on LED
    
    
    //While the other ARDUINO hasn't registered my LED's influence on him, keep LED on
    int i=0;
    while(couplings[1][0]==0){
        //turn on LED
        analogWrite(analogOutPin,250);
        
        //if it's the first time we're turnign on the LED, then tell the other ARDUINO and register couplings[0][0]
        if (i==0)
        {
          delay(200); //for stability

          //calculate my influence in myself
          point11 = convertToLux(analogRead(analogInPin));
          //DEBUG
//          Serial.print("point11:");
//          Serial.println(point11);
          couplings[0][0]=point11/percent;

          //send other node my influence in myself, this also indicates my LED is on
          sendDouble(couplings[0][0],hisSlaveAddress,'1');
          i=1;
        }

    }

    //turn off LED to calculate the other node's influence in me
    analogWrite(analogOutPin,0);

    //wait for other node to send me his influence on himself which indicates his LED is on
    while(couplings[1][1]==0){delay(100);}

    //DEBUG
//    Serial.println("left the loop!!!!");

    //Calculate the other node's influence on me
    point12 = convertToLux(analogRead(analogInPin));
    couplings[0][1]=point12/percent;
    
    //send to the other node 
    sendDouble(couplings[0][1],hisSlaveAddress,'2');

    //send the offsets to RPI
    char auxMessage[16];
    for(int i=0;i<16;i++){auxMessage[i]='f';}
    auxMessage[0]='o';
    
    String offset1String=String(illuminationOffset[0],3);
    String offset2String=String(illuminationOffset[1],3);
    
    char offset1Char[offset1String.length()];
    int offset1Size=sizeof(offset1Char);
    offset1String.toCharArray(offset1Char,offset1String.length());   

    char offset2Char[offset2String.length()];
    int offset2Size=sizeof(offset2Char);
    offset2String.toCharArray(offset2Char,offset2String.length());

    for(int i=0;i<offset1Size-1;i++){auxMessage[i+1]=offset1Char[i];}
    for(int i=0;i<offset2Size-1;i++){auxMessage[i+offset1Size]=offset2Char[i];}
    
    int RPIaddress=0x48;

    //DEBUG
    //Serial.println(auxMessage);
    
    Wire.beginTransmission(RPIaddress);
    Wire.write(auxMessage);
    Wire.endTransmission();
    

    //DEBUG
    //Serial.print("I got couplings[1][1] as: ");
    //Serial.print(couplings[1][1]);
    //Serial.print("\t and couplings[0][1] as:");
    //Serial.println(couplings[0][1]);

    
    //Serial.print("I got couplings[0][0] as: ");
    //Serial.print(couplings[0][0]);
    //Serial.print("\t and couplings[1][0] as:");
    //Serial.println(couplings[1][0]);

    //Serial.print("I got offset[0] as: ");    
    //Serial.print(illuminationOffset[0]);
    //Serial.print("\t and offset[1] as:");
    //Serial.println(illuminationOffset[1]);
  }else
  {
    //ARDUINO #11

    //switch off LED
    analogWrite(analogOutPin, 0);
    delay(100); // wait for stability

//    sendDouble(double(1),hisSlaveAddress,'s');//send the first time. If the other arduino didn't receive, it's necessary to keep sending
//    while(startCC==false){sendDouble(double(1),hisSlaveAddress,'s');}//tell other arduino my LED is off and I'm ready to start, wait to have permission to start
//    startCC=false;//reset startCC flag
    
    //wait for other node to calculate his offset
    while(illuminationOffset[1]==0){delay(100);}
    illuminationOffset[0] = convertToLux(analogRead(analogInPin)); //calculate offset
    sendDouble(illuminationOffset[0],hisSlaveAddress,'3'); //send to other node
    
    //wait for other node to send me his influence on himself which indicates his LED is on
    while(couplings[1][1]==0){delay(100);}
    
    //DEBUG
//    Serial.println("left the loop!!!!");

    //Calculate the other node's influence on me
    point12 = convertToLux(analogRead(analogInPin));
    couplings[0][1]=point12/percent;
    
    //send to other node
    sendDouble(couplings[0][1],hisSlaveAddress,'2');


    //While the other ARDUINO hasn't registered my LED's influence on him, keep LED on
    int i=0;
    while(couplings[1][0]==0){
        analogWrite(analogOutPin,250);
        
        //if it's the first time we're turnign on the LED, then tell the other ARDUINO and register couplings[0][0]
        if (i==0)
        {
          delay(200); //for stability
          point11 = convertToLux(analogRead(analogInPin));

          //DEBUG
//        Serial.print("point11:");
//          Serial.println(point11);
          
          //calculate my influence on myself, send to other node. this indicates my LED is on
          couplings[0][0]=point11/percent;
          sendDouble(couplings[0][0],hisSlaveAddress,'1');
          i=1;
        }

    }

    //turn off LED
    analogWrite(analogOutPin,0);

    //send my offset to other node
    sendDouble(illuminationOffset[0],hisSlaveAddress,'3');
    while(illuminationOffset[1]==0){delay(100);}
    
    //DEBUG
    //Serial.print("I got couplings[1][1] as: ");
    //Serial.print(couplings[1][1]);
    //Serial.print("\t and couplings[0][1] as:");
    //Serial.println(couplings[0][1]);
    //Serial.print("I got couplings[0][0] as: ");
    //Serial.print(couplings[0][0]);
//    Serial.print("\t and couplings[1][0] as:");
//    Serial.println(couplings[1][0]);
//    Serial.print("I got offset[0] as: ");
//    Serial.print(illuminationOffset[0]);
//    Serial.print("\t and offset[1] as:");
//    Serial.println(illuminationOffset[1]);
  }


}

//Determine FF's response based on the solution calculated in the consensus
void Control_System::generateFFOutput(double setpoint)
{
  outputFF=dConsensus[0]*2.55;
}

//Check if system is in the deadzone,1 if it is
bool Control_System::checkDeadZone ()
{
  double limError = 3.5;//deadzone size
  return ((abs(error) < limError));//for comfort reasons, the illuminance should be > lowerBound
}

void Control_System::consensus()
{
  
  //To compare to the .mat test, it is required to change matrix L and illuminationOffset 
  //if(hisSlaveAddress==10){int aux=L[0];L[0]=L[1];L[1]=aux;aux=illuminationOffset[0];illuminationOffset[0]=illuminationOffset[1];illuminationOffset[1]=aux;}
  
  
  double rho=0.01;

  //INITILIZATION
  double d[2]={0,0};
  double dAverage[2]={0,0};
  d_his[1]=0; d_his[0]=0; //copy of the other arduino's last solution
  double y[2]={0,0};

  //ITERATIONS
  for(int iteration=0; iteration<50; iteration++)
  {
      //Make node wait here for the other to finish it's iteration (arduino #10 starts) 
      if ( (hisSlaveAddress!=11) || (iteration!=0) ){while(!nextIteration){delay(100);}}
      //Reset nextIteration flag
      nextIteration=0;

//      Serial.print("ITERATION #");
//      Serial.println(iteration);
      //debug( d_his,  iteration, 2, "d_his");
      //debug( y,  iteration, 2, "y");
      double d_best[2]={-1,-1};
      double min_best=100000;
      bool sol_unconstrained=1, sol_boundary_linear=1, sol_boundary_0=1, sol_boundary_100=1, sol_linear_0=1, sol_linear_100=1;
      double z[2]={-c[0]-y[0]+rho*dAverage[0],-y[1]+rho*dAverage[1]};
      //debug( z,  iteration, 2, "z");
//      if(iteration==0){
//        Serial.print("z=[");
//        Serial.print(z[0]);
//        Serial.print(",");
//        Serial.print(z[1]);
//        Serial.println("]");
//      }
      double u[3]={illuminationOffset[0]-L[0],0,100};
      //debug( u,  iteration, 3, "u");
      double p[2]={1/(rho+Q[0][0]),1/rho};
      //debug( p,  iteration, 2, "p");
      double n=couplings[0][0]*couplings[0][0]*p[0]+couplings[0][1]*couplings[0][1]*p[1];
      //debug( &n,  iteration, 0, "n");
      double w[3]={-couplings[0][0]*p[0]*z[0]-couplings[0][1]*p[1]*z[1],-z[0]*p[0],z[0]*p[0]};
      //debug( w,  iteration, 3, "w");

      
      
      //compute unconstrained minimum
      double du[2]={  p[0]*z[0] , p[1]*z[1] };
      //debug( du,  iteration, 2, "du");

      //check feasibility of unconstraines minimu using local constraints
      if( (du[0]<0) || (du[0]>100)  ||  couplings[0][0]*du[0]+couplings[0][1]*du[1]<L[0]-illuminationOffset[0] ){sol_unconstrained=0;}
      if(sol_unconstrained)
      {
          double min_unconstrained=systemCostFunction(du, y, dAverage, rho);
          //debug( &min_unconstrained,  iteration, 0, "min_unconstrained");
          if(min_unconstrained<min_best)
          {
            d_best[0]=du[0];
            d_best[1]=du[1];
            min_best=min_unconstrained;
          }
      }


      
      
      //Compute minimum constrained to linear boundary   
      double db1[2] = {p[0]*z[0]+p[0]*couplings[0][0]/n*(w[0]-u[0]) , p[1]*z[1]+p[1]*couplings[0][1]/n*(w[0]-u[0])};
      //debug( db1,  iteration, 2, "db1");
      //check feasibility of unconstraines minimu using local constraints
      if( (db1[0]<0) || (db1[0]>100) ){sol_boundary_linear=0;}
      if(sol_boundary_linear)
      {
          double min_boundary_linear=systemCostFunction(db1, y, dAverage, rho);
          //debug( &min_boundary_linear,  iteration, 0, "min_boundary_linear");
          if(min_boundary_linear<min_best)
          {
            d_best[0]=db1[0];
            d_best[1]=db1[1];
            min_best=min_boundary_linear;
          }
      }

      
      
      
      //Compute minimum constrained to 0 boundary   
      double db0[2] = {0 , p[1]*z[1]};
      //debug( db0,  iteration, 2, "db0");

      //check feasibility of unconstraines minimu using local constraints
      if(  (couplings[0][0]*db0[0]+couplings[0][1]*db0[1]<L[0]-illuminationOffset[0]) || (db0[0]>100) ){sol_boundary_0=0;}
      if(sol_boundary_0)
      {
          double min_boundary_0=systemCostFunction(db0, y, dAverage, rho);
          //debug( &min_boundary_0,  iteration, 0, "min_boundary_0");
          if(min_boundary_0<min_best)
          {
            d_best[0]=db0[0];
            d_best[1]=db0[1];
            min_best=min_boundary_0;
          }
      }


      
      //Compute minimum constrained to 100 boundary   
      double db100[2] = {100 , p[1]*z[1]};
      //debug( db100,  iteration, 2, "db100");
      //check feasibility of unconstraines minimu using local constraints
      if(  (couplings[0][0]*db100[0]+couplings[0][1]*db100[1]<L[0]-illuminationOffset[0]) || (db100[0]<0) ){sol_boundary_100=0;}
      if(sol_boundary_100)
      {
          double min_boundary_100=systemCostFunction(db100, y, dAverage, rho);
          //debug( &min_boundary_100,  iteration, 0, "min_boundary_100");
          if(min_boundary_100<min_best)
          {
            d_best[0]=db100[0];
            d_best[1]=db100[1];
            min_best=min_boundary_100;
          }
      }



      //Compute minimum constrained to linear and 0 boundary   
      double common = (rho+Q[0][0])/((rho+Q[0][0])*n-couplings[0][0]*couplings[0][0]);
      double det1 = common;
      double det2 = -couplings[0][0]*common;
      double det3 = det2;
      double det4 = n*(rho+Q[0][0])*common;
      double x1 = det1*w[0] + det2*w[1];
      double x2 = det3*w[0] + det4*w[1];
      double v1 = det1*u[0];
      double v2 = det3*u[0];
      double d10[2] = {p[0]*z[0]+p[0]*couplings[0][0]*(x1-v1)+p[0]*(x2-v2)  , p[1]*z[1]+p[1]*couplings[0][1]*(x1-v1) };
      //debug( d10,  iteration, 2, "d10");
      //check feasibility of unconstraines minimu using local constraints
      if( (d10[0]>100) ){sol_linear_0=0;}
      if(sol_linear_0)
      {
          double min_linear_0=systemCostFunction(d10, y, dAverage, rho);
          //debug( &min_linear_0,  iteration, 0, "min_linear_0");
          if(min_linear_0<min_best)
          {
            d_best[0]=d10[0];
            d_best[1]=d10[1];
            min_best=min_linear_0;
          }
      }


     
      
      //Compute minimum constrained to linear and 0 boundary   
      common = (rho+Q[0][0])/((rho+Q[0][0])*n-couplings[0][0]*couplings[0][0]);
      det1 = common;
      det2 = couplings[0][0]*common;
      det3 = det2;
      det4 = n*(rho+Q[0][0])*common;
      x1 = det1*w[0] + det2*w[2];
      x2 = det3*w[0] + det4*w[2];
      v1 = det1*u[0] + det2*u[2];
      v2 = det3*u[0] + det4*u[2];
      double d1100[2] = {p[0]*z[0]+p[0]*couplings[0][0]*(x1-v1)-p[0]*(x2-v2)  , p[1]*z[1]+p[1]*couplings[0][1]*(x1-v1) };
      //debug( d1100,  iteration, 2, "d1100");
      //check feasibility of unconstraines minimu using local constraints
      if( (d1100[0]>100) ){sol_linear_100=0;}
      if(sol_linear_100)
      {
          double min_linear_100=systemCostFunction(d1100, y, dAverage, rho);
          //debug( &min_linear_100,  iteration, 0, "min_linear_100");
          if(min_linear_100<min_best)
          {
            d_best[0]=d1100[0];
            d_best[1]=d1100[1];
            min_best=min_linear_100;
          }
      }

      //depending on which Arduino it is, it will act differently
      int coord1=11-hisSlaveAddress; 
      int coord2=hisSlaveAddress-10;

      
      //compute average with available knowledge
      //d_best has in [0] information regarding it's own desk and on [1] information regarding the other's desk. (P1)
      //dAverage has in [0] information regarding Arduino #10 desk and on [1] information regarding #11 desk since all the code in the .mat file was done with this same prespective (P2)
      dAverage[0] = (d_best[coord1]+d_his[0])/2;
      dAverage[1] = (d_best[coord2]+d_his[1])/2;
      
      //debug( dAverage,  iteration, 2, "dAverage");
      //update local lagrangian
      //once again, it is necessary to save the information as P1, knowing that d_best and yPrev follow P2
      double yPrev[2]={y[0],y[1]};
      y[0] = yPrev[coord1] + rho*(d_best[coord1]-dAverage[0]);
      y[1] = yPrev[coord2] + rho*(d_best[coord2]-dAverage[1]);
      //debug( y,  iteration, 2, "y");

      //y and dAverage need to be converted to P2
      if(hisSlaveAddress==10){double aux1=dAverage[0];dAverage[0]=dAverage[1];dAverage[1]=aux1;double aux2=y[0];y[0]=y[1];y[1]=aux2;}
      
      //debug( dAverage,  iteration, 2, "dAverage");
      //debug( y,  iteration, 2, "y");
      // send solution to neighboors as P1
      sendDouble(d_best[coord1],hisSlaveAddress,'4');
      delay(5);
      sendDouble(d_best[coord2],hisSlaveAddress,'5');
            
  }

   if(hisSlaveAddress==11){while(!nextIteration){delay(100);}}

   //DEBUG
   //Serial.print("d=[");
   //Serial.print(dAverage[0]);
   //Serial.print(",");
   //Serial.print(dAverage[1]);
   //Serial.print("] \t L=[");
   setpoint=couplings[0][0]*dAverage[0]+couplings[0][1]*dAverage[1]+illuminationOffset[0];
   //Serial.print(setpoint);
   //Serial.print(",");
   double aux=couplings[1][0]*dAverage[0]+couplings[1][1]*dAverage[1]+illuminationOffset[1];
   //Serial.print(aux);
   //Serial.print("]");

   //dConsensus has in [0] the calculated output for it's own desk and in [1] the output for the other.
   dConsensus[0]=dAverage[0];
   dConsensus[1]=dAverage[1];
}

//calculate the cost of the solution d, returns the cost 
double Control_System::systemCostFunction(double d[2], double y[2], double dAverage[2], double rho)
{
  double cost=0.5*Q[0][0]*d[0]*d[0]+c[0]*d[0] + y[0]*(d[0]-dAverage[0]) + y[1]*(d[1]-dAverage[1]) + rho*(d[0]-dAverage[0])*(d[0]-dAverage[0])/2 + rho*(d[1]-dAverage[1])*(d[1]-dAverage[1])/2;
  return cost;
}

//DEBUG
void Control_System::debug(double* variable, int iteration, int howLong, char* varName)
{
   if(iteration<3){
    //Serial.print(varName);   
    //Serial.print("=[");
    for(int i=0;i<howLong;i++)
    {
        //Serial.print(variable[i]);
        //Serial.print(",");
    }
    //Serial.println("]");
   }
}

double Control_System::getSetpoint()
{
   return setpoint;
}

double Control_System::getIlluminationOffset()
{
    return illuminationOffset[0];
}

double Control_System::getLowerBound(int index)
{
  return L[index];
}

void Control_System::checkTaskQueue(int analogInPin, int analogOutPin, int hisSlaveAddress, double initialSetpoint)
{
  if(taskQ==1){
    analogWrite(analogOutPin, 0); 
    
    sendDouble(double(1),hisSlaveAddress,'9');
    while(resetReady==false){delay(1);}//delay(100)
    resetReady=false;
    if(hisSlaveAddress==11){delay(1000);}//wait for stability

    //reset L, couplings and illumination offset
    changeLowerBound(initialSetpoint, 1);
    L[1]=initialSetpoint;
    for(int i=0; i<2; i++)
    {
      illuminationOffset[i]=0;
      for(int j=0; j<2; j++)
      {
        couplings[i][j]=0;
      }
    }
    
    calibration(analogInPin, analogOutPin, hisSlaveAddress);
    consensus();
    taskQ=0;
  }
  if(taskQ==2){
    consensus();
    taskQ=0;
  }
}

void Control_System::changeSetPoint(double setp)
{
   setpoint=(setp>0?setp:setpoint);
  
  //turn off FB
  FBon=0;
  
  //FB needs to be reinitialized
  initialize = true;

  initializeTimeStamp=micros();
}

double Control_System::systemCostFunction2()
{
  double cost=(dConsensus[0]*dConsensus[0])*Q[0][0]+(dConsensus[1]*dConsensus[1])*Q[1][1]+c[0]*dConsensus[0]+c[1]*dConsensus[1];
  return cost;
}

