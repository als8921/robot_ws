import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

import PythonLibMightyZap_PC as MightyZap

port = '/dev/ttyUSB0'
baudrate = 57600
Actuator_ID = 3

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('rcs_command')
        self.cam_cover_subscriber = self.create_subscription(Bool, 'rcs/cam_cover', self.cam_cover_cb, 10)

    def cam_cover_cb(self, msg):
        """
            스트로크 위치를 open_length 위치로 이동
        """
        # 이동하고자 하는 위치 (0~4095)
        open_length = 1000

        length_d = open_length * int(msg.data)
        print(length_d)
        MightyZap.OpenMightyZap(port,baudrate)
        self.get_logger().info(f"position command : {length_d}")
        MightyZap.GoalPosition(Actuator_ID, length_d)

        # 지정한 위치와 현재 위치의 차 값이 Boundary 미만일 경우 서보 액츄에이터의 Force를 강제적으로 비활성
        while(abs(MightyZap.PresentPosition(Actuator_ID) - length_d) > 10):
            print(MightyZap.PresentPosition(Actuator_ID))

        # 서보 액츄에이터의 Force를 강제적으로 비활성
        MightyZap.ForceEnable(Actuator_ID, 0)

def main(args=None):
    rclpy.init(args=args)
    commandSubscriber = CommandSubscriber()
    rclpy.spin(commandSubscriber)
    commandSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
