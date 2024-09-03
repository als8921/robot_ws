import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('rcs_cam_cover_publisher')
        self.cam_cover_publisher = self.create_publisher(Bool, 'rcs/cam_cover', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 콜백 호출

    def timer_callback(self):
        cam_cover_msg = Bool()
        cam_cover_msg.data = False  # 카메라 커버 열기
        self.cam_cover_publisher.publish(cam_cover_msg)
        self.get_logger().info(f'Published cam_cover: {cam_cover_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    commandPublisher = CommandPublisher()
    rclpy.spin(commandPublisher)
    commandPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
