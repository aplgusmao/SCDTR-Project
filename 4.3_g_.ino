const int analogInPin = A0; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
int sensorValue = 0; // value read from the pot
int outputValue = 100; // value output to the PWM (analog out)
double sensorValueLux = 0;
double sensorValueVolts = 0;
float a = 0.0;
double b = 0.0;
double c = 0.0;
double d = 0.0;
double e = 0.0;

void setup() 
{
  Serial.begin(230400); // initialize serial communications at 9600 bps
}

void loop() 
{
  sensorValue = analogRead(analogInPin); // read the analog in value
  a = pow(10.0, 8.0/3.0);
  b = 1023.0/sensorValue;
  c = b-1;
  d = 10*c;
  e = pow(d, 4.0/3.0);
  sensorValueLux = a/e;
  sensorValueVolts = 5.0*sensorValue/1023;

  // print the results to the Serial Monitor:
  //Serial.print("sensor in Lux= ");
  Serial.println(sensorValueLux);

  analogWrite(analogOutPin, outputValue); // change the analog out value
  
    
  delay(1); // wait 2 milliseconds before the next loop for the
  
  //analog-to-digital converter to settle after the last reading
}
