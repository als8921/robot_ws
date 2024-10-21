import rclpy
from rclpy.node import Node
from lift_srv.srv import LiftCommand

class CommandClient(Node):
    def __init__(self):
        super().__init__('command_client')
        self.client = self.create_client(LiftCommand, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')

    def send_request(self, command, value):
        request = LiftCommand.Request()
        request.command = command
        request.value = value
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    command_client = CommandClient()

    command, value = "MOVE", 1.5
    response = command_client.send_request(command, value)
    if response is not None:
        command_client.get_logger().info(f'{command} + {value} = {response.status}')
    else:
        command_client.get_logger().error('결과를 받을 수 없습니다.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
