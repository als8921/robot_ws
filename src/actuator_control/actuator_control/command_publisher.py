import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import time

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('rcs_CommandPublisher')
        self.count = 0
        self.rail_refpos_publisher = self.create_publisher(Int32, 'rail_refpos', 10)
        self.rail_refvel_publisher = self.create_publisher(Int32, 'rail_refvel', 10)

        self.rail_emg_publisher = self.create_publisher(Bool, 'rcs/rail_emg', 10)
        self.rail_calib_publisher = self.create_publisher(Bool, 'rcs/rail_calib', 10)
        self.cam_cover_publisher = self.create_publisher(Bool, 'rcs/cam_cover', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # 1초마다 콜백 호출

    def timer_callback(self):
        self.count += 1
        rail_refpos_msg = Int32()
        rail_refpos_msg.data = self.count*100  # 원하는 위치
        self.rail_refpos_publisher.publish(rail_refpos_msg)
        self.get_logger().info(f'Published rail_refpos: {rail_refpos_msg.data}')

        # rail_refvel_msg = Int32()
        # rail_refvel_msg.data = 100  # 원하는 속도
        # self.rail_refvel_publisher.publish(rail_refvel_msg)
        # self.get_logger().info(f'Published rail_refvel: {rail_refvel_msg.data}')

        # rail_emg_msg = Bool()
        # rail_emg_msg.data = True  # 정지 신호
        # self.rail_emg_publisher.publish(rail_emg_msg)
        # self.get_logger().info(f'Published rail_emg: {rail_emg_msg.data}')

        # rail_calib_msg = Bool()
        # rail_calib_msg.data = False  # 보정 신호
        # self.rail_calib_publisher.publish(rail_calib_msg)
        # self.get_logger().info(f'Published rail_calib: {rail_calib_msg.data}')

        # cam_cover_msg = Bool()
        # cam_cover_msg.data = True  # 카메라 커버 열기
        # self.cam_cover_publisher.publish(cam_cover_msg)
        # self.get_logger().info(f'Published cam_cover: {cam_cover_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    commandPublisher = CommandPublisher()
    rclpy.spin(commandPublisher)
    commandPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
