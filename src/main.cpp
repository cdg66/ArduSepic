#include <Arduino.h>
#include <PID_v1.h>

#define PIN_V_OUTPUT 1
#define PIN_I_OUTPUT 2

#define PIN_V_INPUT 3
#define PIN_I_INPUT 4

#define PIN_T_INPUT 5

#define PIN_MOSFET 6



//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN_V_INPUT);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_V_INPUT);
  myPID.Compute();
  analogWrite(PIN_MOSFET, Output);
}
