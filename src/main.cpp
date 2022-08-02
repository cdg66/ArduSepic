#include <Arduino.h>
#include <PID_v1.h>

#define PIN_V_OUTPUT 1
#define PIN_I_OUTPUT 2

#define PIN_V_INPUT 3
#define PIN_I_INPUT 4

#define PIN_T_INPUT 5

#define PIN_MOSFET 6

//#define PRINT // definition pour allow les print comment out to dont output to the com port

//function prototype
float calculTemp(int16_t sensorValue);
void TempCheck(void);
void EficiencyCheck(uint16_t Vin, uint16_t Iin, uint16_t Vout, uint16_t Iout);
//Define Variables we'll be connecting to
double SetpointV = 511, InputV, OutputV; // 511 = 1023/2 car a 12v on a 2.5V soit Vref/2
double SetpointI, InputI, OutputI; //TODO setpoint current a determiner

//Specify the links and initial tuning parameters
double VKp=2, VKi=5, VKd=1; // TODO connst a determiner
double IKp=2, IKi=5, IKd=1;
PID VPID(&InputV, &OutputV, &SetpointV, VKp, VKi, VKd, DIRECT);
PID IPID(&InputI, &OutputI, &SetpointI, IKp, IKi, IKd, DIRECT);

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT); //led
  digitalWrite(13, 0);
  //initialize the variables we're linked to
  InputV = analogRead(PIN_V_INPUT);
  InputI = analogRead(PIN_I_INPUT);



  //turn the PID on
  VPID.SetOutputLimits(0, ((255*72)/100)); // set the max pwm at 72% so not to burn some mosfet again!
  VPID.SetMode(AUTOMATIC);
  IPID.SetMode(AUTOMATIC);
}

void loop()
{
  TempCheck();
  // Reg Tension
  InputV = analogRead(PIN_V_INPUT);
  VPID.Compute();
  analogWrite(PIN_MOSFET, OutputV);
  
}

float calculTemp(int16_t sensorValue)
{
  float tensionArduino = (sensorValue * 5.0)/1023.0; 
  
  
  float diviseurTension = tensionArduino/2.0;
  
  float R2 = ((diviseurTension/5.0)*1500.0)/(1.0-diviseurTension/5.0);
  #ifdef PRINT
  Serial.print("R2: ");
  Serial.print(R2);
  #endif
  float temp = 0.0;
  temp =  -25.87*log(R2)+225.57;
  #ifdef PRINT
  Serial.print("\tTemp: ");
  Serial.println(temp);
  #endif
  //delay(100);
  return temp;
}

void TempCheck(void)
{
  if(calculTemp(analogRead(PIN_I_INPUT))>40.0)
  {
    analogWrite(PIN_MOSFET, 0); // power off the mosfet 
    digitalWrite(13, 0); // led on to indicate temp fault
    while(1)
    {

    }
  }
}

void EficiencyCheck(uint16_t Vin, uint16_t Iin, uint16_t Vout, uint16_t Iout)
{
  uint32_t Pin  = Vin*Iin;
  uint32_t Pout = Vout*Iout;

  float Eff = Pout/(Pin+Pout);

  #ifdef PRINT
  Serial.print("\tEfficency: ");
  Serial.println(Eff);
  #endif
}