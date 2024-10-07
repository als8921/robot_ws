import serial
import time

# 아두이노가 연결된 포트와 보드레이트 설정
port = '/dev/ttyACM0'  # Windows의 경우 COM 포트, Linux의 경우 /dev/ttyUSB0 등
baudrate = 57600

# 시리얼 포트 열기
ser = serial.Serial(port, baudrate)

# 데이터 수신 대기
time.sleep(2)  # 아두이노와의 연결이 안정될 때까지 대기

try:
    while True:
        if ser.in_waiting > 0:  # 수신 데이터가 있을 경우
            line = ser.readline().decode('utf-8').rstrip()  # 데이터 읽기
            print(f"받은 데이터: {line}")  # 데이터 출력
            
            # 데이터를 쉼표로 분리하여 변수에 저장
            stateL, stateO, stateR = map(int, line.split(','))
            print(f"핀 상태: 핀1: {stateL}, 핀2: {stateO}, 핀3: {stateR}")

except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    ser.close()  # 시리얼 포트 닫기
