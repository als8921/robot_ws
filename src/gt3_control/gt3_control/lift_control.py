import time
from enum import Enum
import threading
import serial
import serial.rs485
from .submodules.MD400 import MD400
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lift_srv.srv import LiftCommand  # 서비스 타입에 맞게 수정

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 19200
class Status(Enum):
    Progress = "Progress",
    Done = "Done",
    Waiting = "Waiting"

class RS485Communication:
    def __init__(self):
        self.ser = serial.rs485.RS485(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        self.ser.rs485_mode = serial.rs485.RS485Settings()
        self.command_queue = deque()
        self.lift_position = None
        self.status = Status.Waiting
        self.cmd_position = None

    def read_data(self):
        if self.ser.readable():
            response = bytearray()
            response.extend(self.ser.read())
            if response and response[0] == md400.TMID:
                response.extend(self.ser.read(4))
                additional_read = response[-1] + 1
                response.extend(self.ser.read(additional_read))
                # print("Received:    ", *response)

                if response[3] == 197:
                    self.lift_position = md400.bytes_to_pos(response)

                # Test Case : 아두이노 테스트 용도
                # elif(response[5] == 197):
                #     self.lift_position = 1.5
                    
                else:
                    self.lift_position = None

    def send_data(self, data):
        try:
            self.ser.write(data)
            # print("Sent         ", *data)
        except Exception as e:
            print(f"Error sending data: {e}")
    
    def run(self):
        while True:
            if self.command_queue:
                packet = self.command_queue.popleft()
                self.send_data(packet)
                self.read_data()
            else:
                packet = md400.get_pos()
                self.send_data(packet)
                self.read_data()
                # if(self.lift_position != None):
                #     print(self.lift_position)

                if(self.cmd_position != None):
                    if(abs(self.cmd_position - self.lift_position) < 0.05):
                        self.status = Status.Done
                time.sleep(0.1)

    def close(self):
        self.ser.close()

class LiftServiceServer(Node):
    def __init__(self, rs485_comm):
        super().__init__('lift_service_server')
        self.rs485_comm = rs485_comm
        self.srv = self.create_service(LiftCommand, 'lift_command', self.handle_set_position)


    def handle_set_position(self, request, response):
        print(request.command)
        if(request.command == "MOVE"):
            self.get_logger().info(f'Setting position to: {request.value}')
            self.rs485_comm.cmd_position = request.value
            self.rs485_comm.command_queue.append(md400.set_pos(request.value))

            while(self.rs485_comm.status != Status.Done):
                pass
            response.status = True
            self.rs485_comm.status = Status.Waiting

        elif(request.command == "STOP"):
            self.rs485_comm.command_queue.append(md400.stop())
            self.get_logger().info('STOP')
            response.status = True

        elif(request.command == "RESET"):
            self.rs485_comm.command_queue.append(md400.set_alarm_reset())
            self.get_logger().info('ALARM RESET')
            response.status = True

        elif(request.command == "HOMING"):
            self.rs485_comm.command_queue.append(md400.homing())
            self.get_logger().info('HOMING')
            response.status = True

        elif(request.command == "SETRPM"):
            self.rs485_comm.command_queue.append(md400.set_rpm(request.value))
            self.get_logger().info(f'SET RPM to {request.value}[rpm]')
            response.status = True

        elif(request.command == "SETCURRENT"):
            self.rs485_comm.command_queue.append(md400.set_current_limit(request.value))
            self.get_logger().info(f'SET CURRENT to {request.value}[A]')
            response.status = True
        else:
            response.status = False
        return response

def main(args=None):
    rclpy.init(args=args)

    global md400
    md400 = MD400(rmid=0xb7, tmid=0xb8, id=0x01)
    
    rs485_comm = RS485Communication()

    # MultiThreadedExecutor를 생성하고 ROS 서비스 서버 추가
    executor = MultiThreadedExecutor()
    
    lift_service_server = LiftServiceServer(rs485_comm)
    executor.add_node(lift_service_server)

    try:
        # RS485 통신을 위한 스레드 시작
        threading.Thread(target=rs485_comm.run, daemon=True).start()

        # ROS 노드 실행
        executor.spin()
    finally:
        rs485_comm.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
