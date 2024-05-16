#define RELAY1_PIN 13
#define RELAY2_PIN 27
#define RELAY3_PIN 26
#define RELAY4_PIN 25
#define RELAY5_PIN 33
#define RELAY6_PIN 32

void setup() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  pinMode(RELAY5_PIN, OUTPUT);
  pinMode(RELAY6_PIN, OUTPUT);

  // กำหนดให้ทุก Relay ปิดก่อนเริ่มต้น
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  digitalWrite(RELAY5_PIN, LOW);
  digitalWrite(RELAY6_PIN, LOW);
}

void loop() {
  // ทำให้ทุก Relay ติดเป็นเวลา 5 วินาที
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);
  digitalWrite(RELAY5_PIN, HIGH);
  digitalWrite(RELAY6_PIN, HIGH);
  delay(5000);

  // ทำให้ทุก Relay ปิด
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  digitalWrite(RELAY5_PIN, LOW);
  digitalWrite(RELAY6_PIN, LOW);
  delay(5000);
}
