#ifndef _CONTROL_SYSTEM_H_
#define _CONTROL_SYSTEM_H_

//#include "feedForwardOutput.h"
//#include "Arduino.h"

#ifndef _LUX_H_
#include "Lux.h"
#endif

//#ifndef _COMMUNICATIONS_H_
//#include "communications.h"
//#endif

//#ifndef _WIRE_H_
//#include <Wire.h>
//#endif

#ifndef _ARDUINO_H_
#include "Arduino.h"
#endif



#define byte uint8_t



class Control_System
{
  private:
    double Kp, Ki, Kd, sampleRate, b, a, Iprev, I, inputPrev, Dprev, error, errorPrev, P, D, setpoint, outputFF, c[2]={1,1}, Q[2][2]={{0,0},{0,0}}, hisSlaveAddress;
    double d_his[2];
    int L[2]={50,50};
    
  public:
    unsigned long initializeTimeStamp=0, computeTimeStamp=0;
    bool nextIteration=0, resetReady=false, distributedControl=true;
    int taskQ=0;
    const int high=50,low=25; //in lux
    double *input, *output, bFF, mFF, staticErrorHigh, staticErrorLow, realHigh, realLow, dConsensus[2]={0,0}, illuminationOffset[2] = {0,0}, couplings[2][2]={{0,0},{0,0}};
    bool FFoff=0,FBon=0,initialize;
    Control_System(double*, double*, double, double, double, double, unsigned long, double, double, int);
    ~Control_System();
    void compute();
    void changeParameters(double, double, double, double, double);
    void changeSampleRate(double);
    void changeLowerBound(double,bool initFlag=0);
    void changeCouplings(int line, int column, double value);
    void changeOffset(int index, double value);
    void changeCopy(int index, double value);
    void changeHisLowerBound(double value);
    void calibration(int, int,int);
    void generateFFOutput(double);
    bool checkDeadZone();
    void consensus();
    double systemCostFunction(double d[2], double y[2], double dAverage[2], double rho);
    void debug(double* variable, int iteration, int howLong, char* varName);
    double getSetpoint();
    double getIlluminationOffset();
    double getLowerBound(int index=0);
    void checkTaskQueue(int analogInPin, int analogOutPin, int hisSlaveAddress, double initialSetpoint);
    void changeSetPoint(double setp);
    double systemCostFunction2();
};


#ifndef _COMMUNICATIONS_H_
#include "Communications.h"
#endif


#endif //_CONTROL_SYSTEM_
