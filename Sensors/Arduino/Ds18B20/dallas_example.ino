// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

// Temperature sensor
#include <DallasTemperature.h>
#define ONE_WIRE_BUS TSAPIN
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

int temp_resolution = 12;
int temp_measureperiod = 5000;  // millis
int temp_convertdelay = 0;  // millis (calculated in setup from resolution)
int temp_idle;
unsigned long temp_lastconvertrequest = 0;
unsigned long temp_lastupdate = 0;
DeviceAddress temp0_DeviceAddress, temp1_DeviceAddress;
float temp0 = -1000.0, temp1 = -1000;
float temp_setpoint = 47.0;
float temp_reqsetpoint = temp_setpoint;

void setup_tempsens(void) 
{ 
  Serial.println("Calling begin on DallasTemperature sensor"); 
  sensors.begin(); 

  int available = sensors.getDeviceCount();
  Serial.print("Sensors available: ");  Serial.println(available, DEC);
  
  sensors.getAddress(temp0_DeviceAddress, 0);
  sensors.getAddress(temp1_DeviceAddress, 1);
  temp_convertdelay = 750 / (1 << (12 - temp_resolution));
  sensors.setResolution(temp0_DeviceAddress, temp_resolution);
  sensors.setResolution(temp1_DeviceAddress, temp_resolution);
 
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  temp_lastconvertrequest = millis();
  temp_idle = 0;  // conversion in progress
} 

void loop_tempsens(void) 
{ 
  // Start new conversion if required
  if( temp_idle==1 ) {
    if( millis()-temp_lastconvertrequest>=temp_measureperiod ) {
      //Log("Requesting temperatures..."); 
      sensors.requestTemperatures();
      temp_lastconvertrequest = millis();
      temp_idle = 0;  // conversion in progress
    } 
    return;
  }

  // Handle ready temperature measurement
  temp0 = sensors.getTempCByIndex(0);  // first device on bus
  if( temp0<0 ) temp0 = 1000;
  temp1 = sensors.getTempCByIndex(1);
  if( temp1<0 ) temp1 = 1000;
  temp_lastupdate = millis();
  //Log("Temperatures: " + String(temp0) + "," + String(temp1) + " (lastupdate=" + temp_lastupdate + ")");

  temp_idle = 1;  //  ready for next
} 
