# add_two_ints_client.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from ros_service.msg import Status
from ros_service.srv import Command

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('서비스를 기다리는 중...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()

    a = 5
    b = 3
    response = add_two_ints_client.send_request(a, b)
    if response is not None:
        add_two_ints_client.get_logger().info(f'{a} + {b} = {response.sum}')
    else:
        add_two_ints_client.get_logger().error('결과를 받을 수 없습니다.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
