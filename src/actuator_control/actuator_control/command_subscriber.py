import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

import PythonLibMightyZap_PC.PythonLibMightyZap_PC as MightyZap

port = '/dev/ttyUSB0'
baudrate = 57600
Actuator_ID = 3

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('rcs_command')
        self.open()
        self.rail_refpos_subscriber = self.create_subscription(Int32, 'rail_refpos', self.rail_refpos_cb, 10)
        self.rail_refvel_subscriber = self.create_subscription(Int32, 'rail_refvel', self.rail_refvel_cb, 10)

        self.rail_emg_subscriber = self.create_subscription(Bool, 'rcs/rail_emg', self.rail_emg_cb, 10)
        self.rail_calib_subscriber = self.create_subscription(Bool, 'rcs/rail_calib', self.rail_calib_cb, 10)
        self.cam_cover_subscriber = self.create_subscription(Bool, 'rcs/cam_cover', self.cam_cover_cb, 10)

    def open(self):
        """
            시리얼 포트 연결
        """
        MightyZap.OpenMightyZap(port,baudrate)


    def rail_refpos_cb(self, msg):
        """
            스트로크 위치를 원하는 위치로 이동
            Args:
                msg : 이동하고자 하는 위치 (0~4095)
        """
        self.get_logger().info(f"position command : {msg.data}")
        MightyZap.GoalPosition(Actuator_ID, msg.data)

        # 지정한 위치와 현재 위치의 차 값이 Boundary 미만일 경우 서보 액츄에이터의 Force를 강제적으로 비활성
        while(abs(MightyZap.PresentPosition(Actuator_ID) - msg.data) > 10):
            print(MightyZap.PresentPosition(Actuator_ID))

        MightyZap.ForceEnable(Actuator_ID, 0)

    def rail_refvel_cb(self, msg):
        pass

    def rail_emg_cb(self, msg):
        """
            서보 액츄에이터의 Force를 강제적으로 활성/비활성
            Args:
                msg : True(활성) / False(비활성)
        """
        MightyZap.ForceEnable(Actuator_ID, int(msg.data))


    def rail_calib_cb(self, msg):
        pass

    def cam_cover_cb(self, msg):
        pass
    




def main(args=None):
    rclpy.init(args=args)
    commandSubscriber = CommandSubscriber()
    rclpy.spin(commandSubscriber)
    commandSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
