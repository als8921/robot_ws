String receivedData = ""; // 데이터를 저장할 변수

void setup() {
  Serial.begin(19200);
}

void loop() {
  while (Serial.available() > 0) {
    char incomingByte = Serial.read(); // 한 바이트씩 읽기
    receivedData += incomingByte; // 읽은 바이트를 누적

    // 예를 들어, 특정 길이(예: 10바이트)가 되면 출력
    if (receivedData.length() >= 10) {
      Serial.print(receivedData); // 저장된 데이터 출력
      receivedData = ""; // 데이터 초기화
    }
  }

  // 추가적인 로직을 여기에 넣을 수 있습니다.
}
