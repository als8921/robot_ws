import serial
import time

# 아두이노가 연결된 포트와 보드레이트 설정
port = '/dev/ttyACM0'  # Windows의 경우 COM 포트, Linux의 경우 /dev/ttyUSB0 등
baudrate = 9600

# 시리얼 포트 열기
ser = serial.Serial(port, baudrate)

# 데이터 수신 대기
time.sleep(2)  # 아두이노와의 연결이 안정될 때까지 대기

while True:
    try:
        if ser.readable():
            data = ser.readline().decode('utf-8').rstrip()  # 데이터 읽기
            # print(data)
            if(len(data) > 0 and data[0] != ","):
                stateL, stateO, stateR = [1 if i > 10 else 0 for i in list(map(int, data.split(',')))]
                print(stateL, stateO, stateR)
    
    except KeyboardInterrupt:
        break

    except:
        pass
