#include "DHT.h"
#include <BH1750FVI.h>
#include <Arduino.h>

// DHT Sensor
#define DHTPIN 15
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

// MQ-7 Sensor
const int mq7AnalogPin = 32;
const int mq7DigitalPin = 16;
float Rs;

// Flow Sensor
#define FLOW_SENSOR_PIN 4
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

// Relay pin definitions
#define RELAY1_PIN 33 // fan
#define RELAY2_PIN 26 // light
#define RELAY3_PIN 13 // alarm
#define RELAY4_PIN 27 // fogger
#define RELAY5_PIN 14 // drip
#define RELAY6_PIN 17 // pump

bool relayStates[] = {LOW, LOW, LOW, LOW, LOW, LOW};

// BH1750 Sensor
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

// Shared variable for sensor data
volatile uint16_t lightIntensity = 0;
volatile float COppm = 0;
volatile float humidity = 0;
volatile float temperature = 0;
volatile float flowRateGlobal = 0;
volatile unsigned long totalMilliLitresGlobal = 0;

// Mutex for shared variable access
SemaphoreHandle_t sensorMutex;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("DHTxx, BH1750, MQ-7, and Flow Sensor test!"));

  // Initialize the DHT sensor
  dht.begin();

  // Set relay pins as outputs
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  pinMode(RELAY5_PIN, OUTPUT);
  pinMode(RELAY6_PIN, OUTPUT);

  // Turn all relays off initially
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  digitalWrite(RELAY5_PIN, LOW);
  digitalWrite(RELAY6_PIN, LOW);

  // Initialize the BH1750 sensor
  LightSensor.begin();

  // Initialize the flow sensor
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // Create a mutex for shared variable access
  sensorMutex = xSemaphoreCreateMutex();

  // Create tasks for the dual-core operation
  xTaskCreatePinnedToCore(
    dhtRelayTask,    // Task function
    "DHT and Relay", // Task name
    10000,           // Stack size
    NULL,            // Parameters
    1,               // Priority
    NULL,            // Task handle
    1);              // Core 1

  xTaskCreatePinnedToCore(
    sensorTask,      // Task function
    "Sensors",       // Task name
    10000,           // Stack size
    NULL,            // Parameters
    1,               // Priority
    NULL,            // Task handle
    0);              // Core 0
}

void loop() {
  // The loop is empty because tasks run on their respective cores
}

// Task for DHT sensor and relay control on Core 1
void dhtRelayTask(void * parameter) {
  while (true) {
    // Wait a few seconds between measurements.
    delay(2000);

    // Reading temperature and humidity
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // Check if reads are successful
    bool validRead = true;
    if (isnan(h) || isnan(t)) {
      validRead = false;
    }

    if (validRead) {
      // Update the shared sensor values
      xSemaphoreTake(sensorMutex, portMAX_DELAY);
      humidity = h;
      temperature = t;
      xSemaphoreGive(sensorMutex);
    }

    // Access the light intensity, CO ppm, and flow rate values
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    uint16_t lux = lightIntensity;
    float co = COppm;
    float flow = flowRateGlobal;
    unsigned long totalFlow = totalMilliLitresGlobal;
    xSemaphoreGive(sensorMutex);

    // Display humidity, temperature, light intensity, CO ppm, and flow rate
    Serial.print(F("H="));
    Serial.print(humidity);
    Serial.print(F(" T="));
    Serial.print(temperature);
    Serial.print(F(" L="));
    Serial.print(lux);
    Serial.print(F(" CO="));
    Serial.print(co, 3);
    Serial.print(F(" Flow="));
    Serial.print(flow);
    Serial.print(F(" L/min Total="));
    Serial.print(totalFlow);
    Serial.println(F(" mL"));

    // Check if data is available in the serial buffer
    if (Serial.available() > 0) {
      // Read the incoming byte
      char incomingByte = Serial.read();

      // Check if the incoming byte is between '1' and '6'
      if (incomingByte >= '1' && incomingByte <= '6') {
        int relayIndex = incomingByte - '1'; // Convert char to index (0-5)

        // Toggle the state of the corresponding relay
        relayStates[relayIndex] = !relayStates[relayIndex];
        digitalWrite(getRelayPin(relayIndex), relayStates[relayIndex]);
      }
    }
  }
}

// Task for reading BH1750, MQ-7, and Flow sensors on Core 0
void sensorTask(void * parameter) {
  while (true) {
    // Read light intensity
    uint16_t lux = LightSensor.GetLightIntensity();

    // Read MQ-7 sensor value
    int sensorValue = analogRead(mq7AnalogPin);
    float coPpm = analysis(sensorValue);

    // Calculate flow rate and total flow
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      pulse1Sec = pulseCount;
      pulseCount = 0;

      flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
      previousMillis = millis();

      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;
    }

    // Update the shared sensor values
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    lightIntensity = lux;
    COppm = coPpm;
    flowRateGlobal = flowRate;
    totalMilliLitresGlobal = totalMilliLitres;
    xSemaphoreGive(sensorMutex);

    delay(250);
  }
}

float analysis(int adc) {
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

int getRelayPin(int index) {
  switch (index) {
    case 0: return RELAY1_PIN;
    case 1: return RELAY2_PIN;
    case 2: return RELAY3_PIN;
    case 3: return RELAY4_PIN;
    case 4: return RELAY5_PIN;
    case 5: return RELAY6_PIN;
    default: return -1; // Should never happen
  }
}
