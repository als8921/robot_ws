import time
import rclpy
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Bool, String
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.InstantDoCtrl import InstantDoCtrl
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.BDaqApi import BioFailed

# 초기 파라미터 설정
DEVICE_DESCRIPTION = "USB-4716,BID#0"
TIMER_PERIOD = 0.01

class STATE(Enum):
    STABLE = "STABLE"
    EMERGENCY = "EMERGENCY"
    LIMIT = "LIMIT"
    HOMING = "HOMING"
    CALIBRATION = "CALIBRATION"
    STEADYSTATE = "STEADYSTATE"

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.instant_do = InstantDoCtrl(DEVICE_DESCRIPTION)
        self.instant_di = InstantDiCtrl(DEVICE_DESCRIPTION)
        self.instant_ao = InstantAoCtrl(DEVICE_DESCRIPTION)
        self.instant_ai = InstantAiCtrl(DEVICE_DESCRIPTION)
        self.instant_ao.channels[0].valueRange = ValueRange.V_Neg10To10
        
        self.current_angle = 0  # 현재 각도 초기화
        self.filtered_velocity = 0  # 필터링된 각속도 초기화
        self.alpha = 0.5            # LPF 필터 계수 (0 < alpha < 1)
        self.offset = 0
        
        # PID 제어 변수
        self.kp = 0.2
        self.kd = 0.025
        self.previous_error = 0
        self.desired_angle = 0  # 목표 각도 초기화
        
        self.State = STATE.CALIBRATION

        self.pos_subscription = self.create_subscription(Int16, '/rcs/rail_refpos', self.pos_callback, 10)
        self.emg_subscription = self.create_subscription(Bool, '/rcs/rail_emg', self.emg_callback, 10)
        self.cali_subscription = self.create_subscription(Bool, '/rcs/rail_calib', self.cali_callback, 10)

        # 시각화를 위한 퍼블리셔 설정
        self.raw_velocity_publisher = self.create_publisher(Float32, '/motor/raw_velocity', 10)
        self.filtered_velocity_publisher = self.create_publisher(Float32, '/motor/filtered_velocity', 10)
        self.angle_publisher = self.create_publisher(Float32, '/motor/angle', 10)
        self.desired_angle_publisher = self.create_publisher(Float32, '/motor/desired_angle', 10)


        self.current_publisher = self.create_publisher(Float32, 'motor/current', 10)
        self.position_publisher = self.create_publisher(String, '/rcs/position', 10)
        self.status_publisher = self.create_publisher(Bool, '/rcs/status', 10)
        
        self.previous_time = time.time()
        # ROS 타이머 설정
        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)

    def calibration(self, calibration_time):
        """ 에러의 Offset을 얻기 위한 Calibration """
        print("Calibration Start")
        print("Do not move the motor.")

        sampling_number = int(calibration_time / TIMER_PERIOD)
        sample_sum = 0
        for _ in range(sampling_number):
            raw_velocity = self.calculate_velocity(self.read_analog()[0])
            sample_sum += raw_velocity
            time.sleep(TIMER_PERIOD)
        return sample_sum / sampling_number

    def cali_callback(self, msg):
        print(msg.data, "Calibration Subscribed")

    def emg_callback(self, msg):
        if(msg.data):
            self.State = STATE.EMERGENCY
        else:
            self.State = STATE.STABLE
    
    def pos_callback(self, msg):
        self.State = STATE.STABLE
        self.desired_position = msg.data                            # unit : [m]
        self.desired_angle = self.desired_position * 720 / 0.4523   # unit : [degree]

    def read_digital(self):
        _, data = self.instant_di.readAny(0, 1)
        length = 3
        digitaldata = [0] * length

        for i in range(length):
            digitaldata[i] = (data[0] >> i) & 1

        return digitaldata
    
    def write_digital(self, data):
        self.instant_do.writeAny(0, 1, [data])
        
    def read_analog(self):
        """ 아날로그 입력을 읽는 함수 """
        ai_channel = 0
        _, analog_data = self.instant_ai.readDataF64(ai_channel, 2)
        return analog_data
    
    def write_analog(self, voltage):
        """ 아날로그 출력을 설정하는 함수 """
        ao_channel = 0
        voltage = max(-10, min(10, voltage))  # 전압 제한
        self.instant_ao.writeAny(ao_channel, 1, None, [-voltage])

    def calculate_velocity(self, voltage):
        """ 주어진 전압에 따라 각속도를 계산하는 함수 """
        return -voltage * 180  # 각속도 계산 (deg/s)

    def pid_control(self, desired_position, current_position, dt):
        """ PID 제어 알고리즘을 사용하여 출력 값을 계산하는 함수 """
        pos_error = desired_position - current_position
        pos_derivative = (pos_error - self.previous_error) / dt
        output = self.kp * pos_error + self.kd * pos_derivative  # i 제어는 사용하지 않음
        self.previous_error = pos_error
        return output

    def control_loop(self):
        """ 제어 루프를 실행하는 함수 """
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        if(self.State == STATE.CALIBRATION):
            self.write_analog(0)
            self.offset = self.calibration(5)
            self.State = STATE.STABLE


        left_limit, origin_limit, right_limit = self.read_digital()
        if(left_limit & right_limit & origin_limit):
            pass
        else:
            # print("LEFT ===== ORIGIN ===== RIGHT")
            # print(f"   {left_limit}          {origin_limit}         {right_limit} ")

            if(left_limit == 0 and right_limit == 0):
                self.State = STATE.STABLE
            if(left_limit == 1):
                self.State = STATE.LIMIT
            if(right_limit == 1):
                self.State = STATE.LIMIT
            if(origin_limit == 1):
                self.current_angle = 0

        analog_input = self.read_analog()
        current = analog_input[1] * 5.9 / 4 # Analog to Current     -4V ~ 4V    :   -5.9A ~ 5.9A
        # Velocity를 받아와서 적분을 통해 angle을 구하는 과정
        raw_velocity = self.calculate_velocity(analog_input[0]) - self.offset
        # Low Pass Filter
        self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
        # 특정 값 사이의 속도를 무시하여 에러 적분 값이 누적되는 것을 막음
        VELOCITY_ERROR_BOUNDARY = 3
        if abs(self.filtered_velocity) < VELOCITY_ERROR_BOUNDARY:
            self.filtered_velocity = 0.0
        self.current_angle += self.filtered_velocity * dt  

        # error값이 0.1 미만일 때 모터를 멈춤
        if(abs(self.current_angle - self.desired_angle) < 0.1):
            self.State = STATE.STEADYSTATE

        """
            STATE에 따른 작업
            ---
            STEADYSTATE : 정상상태로 다음 명령이 들어올 때까지 모터를 정지
            STABLE      : 안정상태로 PID제어 수행
            EMERGENCY   : /rcs/rail_emg 토픽이 True로 들어온 상태로 정지
            LIMIT       : 리밋센서에 인식된 상태로 정지
        """

        if(self.State == STATE.STEADYSTATE):
            self.write_analog(0)
            self.status_publisher.publish(Bool(data = True))
            print("STEADYSTATE")
            
        elif(self.State == STATE.STABLE):
            self.write_digital(0)
            self.status_publisher.publish(Bool(data = False))
            voltage_output = self.pid_control(self.desired_angle, self.current_angle, dt)
            voltage_output = max(-10, min(10, voltage_output)) # 출력 전압 제한
            self.get_logger().info(f"각도: {self.current_angle:.2f}, 목표: {self.desired_angle}, 필터링된 각속도: {self.filtered_velocity:.2f}, 전압: {voltage_output:.2f}")
            self.write_analog(voltage_output)

        elif(self.State == STATE.EMERGENCY):
            print("EMERGENCY")
            self.write_digital(1)
            self.write_analog(0)

        elif(self.State == STATE.LIMIT):
            print("LIMIT")
            self.write_digital(1)
            self.write_analog(0)
        

        current_position = self.current_angle * 0.4523 / 720    # unit : [m]
        self.position_publisher.publish(String(data = str(current_position)))

        # 퍼블리시
        self.raw_velocity_publisher.publish(Float32(data=raw_velocity))
        self.filtered_velocity_publisher.publish(Float32(data=float(self.filtered_velocity)))
        self.angle_publisher.publish(Float32(data=float(self.current_angle)))
        self.desired_angle_publisher.publish(Float32(data=float(self.desired_angle)))
        self.current_publisher.publish(Float32(data=float(current)))

def main(args=None):
    """ ROS 2 노드를 초기화하고 실행하는 메인 함수 """
    rclpy.init(args=args)
    controller = MotorController()
    rclpy.spin(controller)

    try:
        rclpy.init(args=args)
        controller = MotorController()
        rclpy.spin(controller)

    except Exception as e:
        print("Paused:", e)

    finally:
        controller.write_analog(0)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
