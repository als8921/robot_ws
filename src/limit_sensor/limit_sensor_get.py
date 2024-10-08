import serial
import time

class ArduinoReader:
    def __init__(self):
        # 아두이노 연결 설정
        self.port = '/dev/ttyACM0'  # 포트 설정
        self.baudrate = 9600
        self.ser = serial.Serial(self.port, self.baudrate)

        # 데이터 수신 대기
        time.sleep(2)  # 아두이노와의 연결이 안정될 때까지 대기

    def read_data(self):
        try:
            if self.ser.readable():
                data = self.ser.readline().decode('utf-8').rstrip()  # 데이터 읽기
                print(data)
                stateL, stateO, stateR = map(int, data.split(','))
                print(stateL, stateO, stateR)
        except Exception as e:
            print(f"Error reading data: {e}")

def main():
    arduino_reader = ArduinoReader()
    while True:
        arduino_reader.read_data()

if __name__ == '__main__':
    main()
