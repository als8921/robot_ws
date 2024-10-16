void setup() {
  Serial.begin(19200);
}

void loop() {
  // 데이터가 수신되었는지 확인합니다.
  if (Serial.available() > 0) {
    // 수신된 데이터를 읽습니다.
    String receivedData = Serial.readStringUntil('\n');

    // 수신된 데이터를 시리얼 모니터에 출력합니다.
//    Serial.print("Received: ");
    Serial.println(receivedData);
  }
}
