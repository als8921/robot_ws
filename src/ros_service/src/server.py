# add_two_ints_server.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import threading
import time

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        
        # 스레드 생성
        self.thread = threading.Thread(target=self.periodic_task)
        self.thread.start()


        self.thread2 = threading.Thread(target=self.periodic_task2)
        self.thread2.start()

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: a={request.a}, b={request.b}, Sum: {response.sum}')
        return response

    def periodic_task(self):
        while rclpy.ok():
            self.get_logger().info('서버가 작동 중입니다.')
            # time.sleep(1)  # 5초마다 메시지를 출력


    def periodic_task2(self):
        while rclpy.ok():
            self.get_logger().info('123')
            # time.sleep(1)  # 5초마다 메시지를 출력


def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
