import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.BDaqApi import BioFailed

# 초기 파라미터 설정
DEVICE_DESCRIPTION = "USB-4716,BID#0"
TIMER_PERIOD = 0.01

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.instant_ao = InstantAoCtrl(DEVICE_DESCRIPTION)
        self.instant_ai = InstantAiCtrl(DEVICE_DESCRIPTION)
        self.instant_ao.channels[0].valueRange = ValueRange.V_Neg10To10
        
        self.current_angle = 0  # 현재 각도 초기화
        self.filtered_velocity = 0  # 필터링된 각속도 초기화
        self.alpha = 0.5            # 필터 계수 (0 < alpha < 1)
        self.offset = 0
        
        # PID 제어 변수
        self.kp = 10.0
        self.ki = 0     
        self.kd = 1 
        self.previous_error = 0

        self.calibration()
        self.desired_angle = 0  # 목표 각도 초기화
        self.subscription = self.create_subscription(Int16, '/rail/position', self.listener_callback, 10)

        # 필터링된 각속도를 발행하기 위한 퍼블리셔 설정
        self.raw_velocity_publisher = self.create_publisher(Float32, '/motor/raw_velocity', 10)
        self.filtered_velocity_publisher = self.create_publisher(Float32, '/motor/filtered_velocity', 10)
        self.angle_publisher = self.create_publisher(Float32, '/motor/angle', 10)
        self.desired_angle_publisher = self.create_publisher(Float32, '/motor/desired_angle', 10)
        
        self.previous_time = time.time()
        # ROS 타이머 설정
        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)

    def calibration(self):
        """ 에러의 Offset을 얻기 위한 Calibration """
        print("Calibration Start")
        print("Do not move the motor.")

        sampling_number = 5000
        sample_sum = 0
        for _ in range(sampling_number):
            raw_velocity = self.calculate_velocity(self.read_analog())
            self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
            
            sample_sum += raw_velocity
            time.sleep(0.001)
        self.offset = sample_sum / sampling_number
        print(f"Calibration Finish \n Offset: {self.offset}")

    def listener_callback(self, msg):
        """ ROS 2 메시지를 수신하는 콜백 함수 """
        self.desired_angle = msg.data  # 목표 각도 설정

    def read_analog(self):
        """ 아날로그 입력을 읽는 함수 """
        ai_channel = 0
        _, scaled_data = self.instant_ai.readDataF64(ai_channel, 1)
        return scaled_data[0]
    
    def write_analog(self, voltage):
        """ 아날로그 출력을 설정하는 함수 """
        ao_channel = 0
        voltage = max(-10, min(10, voltage))  # 전압 제한
        ret = self.instant_ao.writeAny(ao_channel, 1, None, [-voltage])
        if BioFailed(ret):
            self.get_logger().error("데이터 쓰기 중 오류 발생.")

    def calculate_velocity(self, voltage):
        """ 주어진 전압에 따라 각속도를 계산하는 함수 """
        return -voltage * 180  # 각속도 계산 (deg/s)

    def pid_control(self, desired_position, current_position, dt):
        """ PID 제어 알고리즘을 사용하여 출력 값을 계산하는 함수 """
        pos_error = desired_position - current_position
        pos_derivative = (pos_error - self.previous_error) / dt
        output = self.kp * pos_error + self.kd * pos_derivative  # Ki는 사용하지 않음
        self.previous_error = pos_error
        return output

    def control_loop(self):
        """ 제어 루프를 실행하는 함수 """
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        # Velocity를 받아와서 적분을 통해 angle을 구하는 과정
        raw_velocity = self.calculate_velocity(self.read_analog()) - self.offset
        # Low Pass Filter
        self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity

        # 특정 값 사이의 속도를 무시하여 에러 적분 값이 누적되는 것을 막음
        VELOCITY_ERROR_BOUNDARY = 3
        if abs(self.filtered_velocity) < VELOCITY_ERROR_BOUNDARY:
            self.filtered_velocity = 0.0
        self.current_angle += self.filtered_velocity * dt  

        pid_output = self.pid_control(self.desired_angle, self.current_angle, dt)
        voltage_output = pid_output / 86.73

        # 출력 전압 제한
        voltage_output = max(-10, min(10, voltage_output))
        self.get_logger().info(f"각도: {self.current_angle:.2f}, 목표: {self.desired_angle}, 필터링된 각속도: {self.filtered_velocity:.2f}, 전압: {voltage_output:.2f}")
        
        # 모터에 값 전달
        self.write_analog(voltage_output)

        # 퍼블리시
        self.raw_velocity_publisher.publish(Float32(data=raw_velocity))
        self.filtered_velocity_publisher.publish(Float32(data=float(self.filtered_velocity)))
        self.angle_publisher.publish(Float32(data=float(self.current_angle)))
        self.desired_angle_publisher.publish(Float32(data=float(self.desired_angle)))

def main(args=None):
    """ ROS 2 노드를 초기화하고 실행하는 메인 함수 """
    try:
        rclpy.init(args=args)
        controller = MotorController()
        rclpy.spin(controller)

    except Exception as e:
        print("Paused:", e)
        controller.write_analog(0)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
