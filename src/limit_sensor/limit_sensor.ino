const int limit_L = A0; // 아날로그 포트 A0 사용
const int limit_O = A1; // 아날로그 포트 A1 사용
const int limit_R = A2; // 아날로그 포트 A2 사용

const int numReadings = 5; // 이동 평균을 위한 읽기 수
int readingsL[numReadings]; // 왼쪽 센서 읽기 저장
int readingsO[numReadings]; // 중앙 센서 읽기 저장
int readingsR[numReadings]; // 오른쪽 센서 읽기 저장
int index = 0; // 현재 인덱스
int totalL = 0; // 왼쪽 센서 총합
int totalO = 0; // 중앙 센서 총합
int totalR = 0; // 오른쪽 센서 총합
int averageL = 0; // 이동 평균 값
int averageO = 0; // 이동 평균 값
int averageR = 0; // 이동 평균 값

void setup() {
    Serial.begin(9600);
    // 초기화
    for (int i = 0; i < numReadings; i++) {
        readingsL[i] = 0;
        readingsO[i] = 0;
        readingsR[i] = 0;
    }
}

void loop() {
    // 현재 읽기
    int stateL = analogRead(limit_L);
    int stateO = analogRead(limit_O);
    int stateR = analogRead(limit_R);

    // 왼쪽 센서 이동 평균 계산
    totalL -= readingsL[index]; // 이전 읽기 값 제거
    readingsL[index] = stateL; // 현재 읽기 저장
    totalL += readingsL[index]; // 새로운 읽기 값 추가
    averageL = totalL / numReadings; // 평균 계산

    // 중앙 센서 이동 평균 계산
    totalO -= readingsO[index];
    readingsO[index] = stateO;
    totalO += readingsO[index];
    averageO = totalO / numReadings;

    // 오른쪽 센서 이동 평균 계산
    totalR -= readingsR[index];
    readingsR[index] = stateR;
    totalR += readingsR[index];
    averageR = totalR / numReadings;

    // 인덱스 업데이트
    index = (index + 1) % numReadings; // 인덱스를 순환

    // 상태를 문자열로 변환
    String data = String(averageL) + "," + String(averageO) + "," + String(averageR);
    
    Serial.println(data);  // 시리얼로 전송
}
