#include <Adafruit_ADS1X15.h>

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("START");

  Serial.println("Hello!");
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  ads.begin();
}

void loop()
{
  int16_t adc0, adc1, adc2, adc3;
  Serial.println(" ");
  adc0 = ads.readADC_SingleEnded(0);
  adc0 = adc0 / 25;
  adc1 = ads.readADC_SingleEnded(1);
  adc1 = adc1 / 25;
  adc2 = ads.readADC_SingleEnded(2);
  adc2 = adc2 / 25;
  adc3 = ads.readADC_SingleEnded(3);
  adc3 = adc3 / 25;
  Serial.print("SOIL MOISTURE in percent 1% : "); Serial.println(adc0);
  Serial.print("SOIL MOISTURE in percent 2% : "); Serial.println(adc1);
  Serial.print("SOIL MOISTURE in percent 3% : "); Serial.println(adc2);
  Serial.print("SOIL MOISTURE in percent 4% : "); Serial.println(adc3);
  Serial.println(" ");
  delay(1000);  // Delay for a second before the next loop iteration
}
