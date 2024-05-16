
// SenSor MQ-7 PIN
const int analogPin = 32;
const int digitalPin = 14;

float Rs;
void setup() {
  Serial.begin(115200);
 // pinMode(digitalPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(analogPin);
  Serial.print("adc : ");
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print("Carbon monoxide");
  Serial.print(analysis(sensorValue), 3);
  Serial.print("ppm\n");
  Serial.print("\t");
  Serial.print("SenSor resistance : ");
  Serial.print(Rs);
  delay(500);
}

float analysis(int adc){
  float slope = -0.7516072988;
  float A = 45.87510694;
  float Rseries = 1000;
  float V_Rseries = ((float)adc * 5) / 1023;
  Rs = ((5 - V_Rseries) / V_Rseries) * Rseries;
  float R0 = 400;
  float Y = Rs / R0;
  float CO_ppm = pow(10, (log10(Y / A) / slope));
  return CO_ppm;
}
