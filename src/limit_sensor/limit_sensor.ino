const int limit_L = 2;
const int limit_O = 3;
const int limit_R = 4;

void setup() {
    Serial.begin(57600);
    pinMode(limit_L, INPUT);
    pinMode(limit_O, INPUT);
    pinMode(limit_R, INPUT);
}

void loop() {
    int stateL = digitalRead(limit_L);
    int stateO = digitalRead(limit_O);
    int stateR = digitalRead(limit_R);

    // 상태를 문자열로 변환
    String data = String(stateL) + "," + String(stateO) + "," + String(stateR);
    
    Serial.println(data);  // 시리얼로 전송
}
