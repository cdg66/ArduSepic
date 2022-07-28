#include <Arduino.h>

int Pin = A1; 
int sensorValue = 0;
float calculTemp();
void setup() 
{
  pinMode(Pin, INPUT);
  Serial.begin(9600);
}

void loop() 
{
  //Serial.println(12);
  calculTemp();
}

float calculTemp()
{
  sensorValue = analogRead(Pin);
  float tensionArduino = (sensorValue * 5.0)/1023.0; 
  
  
  float diviseurTension = tensionArduino/2.0;
  
  float R2 = ((diviseurTension/5.0)*1500.0)/(1.0-diviseurTension/5.0);
Serial.print("R2: ");
Serial.print(R2);

 float temp = 0.0;

 
temp =  -25.87*log(R2)+225.57;
Serial.print("\tTemp: ");
Serial.println(temp);
delay(100);
return temp;



}