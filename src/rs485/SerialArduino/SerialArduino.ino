void setup() {
  Serial.begin(19200);
  Serial1.begin(19200);
}

void loop() {
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    Serial1.println(receivedData);
  }



  if (Serial1.available() > 0) {
    String serial1Data = Serial1.readStringUntil('\n');
    Serial.println(serial1Data);
  }
}
