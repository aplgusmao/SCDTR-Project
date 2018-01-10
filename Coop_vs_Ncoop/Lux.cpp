  #ifndef _LUX_H_
#include "Lux.h"
#endif


//Convert sensor value to lux, returns converted value
double convertToLux(double sensorValue)
{
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
  double e = 0.0;
  double sensorValueLux1;

  a = pow(10.0, 8.0/3.0);
  b = 1023.0/sensorValue;
  c = b-1;
  d = 10*c;
  e = pow(d, 4.0/3.0);
  sensorValueLux1 = a/e;
  return sensorValueLux1;
}

