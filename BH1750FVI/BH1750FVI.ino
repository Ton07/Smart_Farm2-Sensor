//  SCL GPIO 22 Brown Line
//  DAT GPIO 21 White Line
//  Vcc 3.3V    Red Line
//  GND         Black Line
//  ADOB        Yellow Line does not need to be connected.

#include <BH1750FVI.h>

BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

void setup() 
{
  Serial.begin(115200);
  LightSensor.begin();  
}

void loop()
{
  uint16_t lux = LightSensor.GetLightIntensity();
  Serial.print("Light: ");
  Serial.println(lux);
  delay(250);
}
