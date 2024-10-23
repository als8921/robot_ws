import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import math

class OdometryListener(Node):
    def __init__(self):
        super().__init__('odometry_listener')
        self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

    def listener_callback(self, msg):
        # 위치 정보 추출
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        # 쿼터니언에서 yaw 각도 계산
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # 결과 출력
        self.get_logger().info(f'Current Position: X: {position_x:.2f}, Y: {position_y:.2f}, Yaw: {math.degrees(yaw):.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    odometry_listener = OdometryListener()
    rclpy.spin(odometry_listener)
    odometry_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
