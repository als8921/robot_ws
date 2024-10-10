import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader_node')
        self.publisher = self.create_publisher(Int16MultiArray, '/rcs/limit_sensor', 10)

        # 아두이노 연결 설정
        self.port = '/dev/ttyACM1'  # 포트 설정
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

                # ROS2 메시지 생성 및 퍼블리시
                msg = Int16MultiArray()
                msg.data = [stateL, stateO, stateR]
                self.publisher.publish(msg)

        except:
            return
def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        while(1):
            arduino_reader.read_data()
    
    except Exception as e:
        print("Exception:", e)
        
    finally:
        arduino_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
