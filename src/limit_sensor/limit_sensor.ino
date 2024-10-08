void setup() {
  // 시리얼 통신 시작
  Serial.begin(9600);
  
  // 8, 9, 10번 핀을 입력으로 설정
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
}

void loop() {
  // 8, 9, 10번 핀의 디지털 값을 읽음
  int value8 = digitalRead(8);
  int value9 = digitalRead(9);
  int value10 = digitalRead(10);
  
  // 하나의 문자열로 만들어 출력
  String output = String(value8) + "," + String(value9) + "," + String(value10);
  Serial.println(output);
}
