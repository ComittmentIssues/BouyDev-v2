#include <Adafruit_INA219.h>
#include <Wire.h>

/*
 * Power consumption measurement sketch
 * 
 * measures the current monitor while the device is active
 * 
 * Parameters: 
 * 
 * F_sample = 1Hz
 * 
 * T_sample = 3 hours
 * 
 * MEasurands:
 * 
 * Bus_Voltage (V), Shunt Voltage(mV), Current(mA), Power (mW)
 */

#define Sample_Time_Seconds 10800 //3 hour sample
 //global

 Adafruit_INA219 ina219;
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200); 
  Serial.println("Power and Current Test");
  ina219.begin();
  ina219.setCalibration_32V_1A();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Bus Voltage (V),Shunt Voltage (mV),Load Voltage (V),Current (mA),Power (mW)");
}


void loop() {
  // put your main code here, to run repeatedly:
    float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;
  int count = 0;

  while(count++ < Sample_Time_Seconds)
  {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print(busvoltage);   Serial.print(",");
  Serial.print(shuntvoltage); Serial.print(",");
  Serial.print(loadvoltage);  Serial.print(",");
  Serial.print(current_mA);   Serial.print(",");
  Serial.print(power_mW);     Serial.println("");
  delay(1000);
  }
  Serial.println("Done");
  while(1)
  {
    
  }
}
