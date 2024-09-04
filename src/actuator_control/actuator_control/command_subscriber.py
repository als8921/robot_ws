import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import PythonLibMightyZap_PC as MightyZap

serial_port = '/dev/ttyUSB0'
baud_rate = 57600
actuator_id = 3

# 이동하고자 하는 위치 (0 ~ 3500)
open_position = 100

# 오차 허용 범위
error_boundary = 10

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('cam_cover_command')
        self.cam_cover_subscriber = self.create_subscription(Bool, 'rcs/cam_cover', self.cam_cover_cb, 10)

        MightyZap.OpenMightyZap(serial_port,baud_rate)
        MightyZap.ForceEnable(actuator_id, 0)

    def cam_cover_cb(self, msg):
        """
            스트로크 위치를 open_position 위치로 이동
            
            target_position = open_position     (True)
            target_position = 0                 (False)
        """

        target_position = open_position * int(msg.data)

        self.get_logger().info(f"position command : {target_position}")
        MightyZap.GoalPosition(actuator_id, target_position)

        # 지정한 위치와 현재 위치의 차이가 error_boundary 미만이 될 때 까지 대기
        while(abs(MightyZap.PresentPosition(actuator_id) - target_position) > error_boundary):
            pass

        # 서보 액츄에이터의 Force를 강제적으로 비활성
        MightyZap.ForceEnable(actuator_id, 0)

def main(args=None):
    try:
        rclpy.init(args=args)
        commandSubscriber = CommandSubscriber()
        rclpy.spin(commandSubscriber)

    except:
        MightyZap.ForceEnable(actuator_id, 0)
        MightyZap.CloseMightyZap()
        commandSubscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
