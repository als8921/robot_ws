import sys, os
sys.path.append(os.path.dirname(__file__))
import time
import rclpy
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Int16MultiArray
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.InstantDoCtrl import InstantDoCtrl

# 초기 파라미터 설정
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUDRATE = 9600
DEVICE_DESCRIPTION = "USB-4716,BID#0"
TIMER_PERIOD = 0.01
POSITION_FILE = os.path.dirname(__file__)+"/current_position.txt"
MAX_CURRENT = 4.5 #[A]

class STATE(Enum):
    CALIBRATION = "CALIBRATION"
    PROCESS = "PROCESS"
    STEADYSTATE = "STEADYSTATE"
    EMERGENCY = "EMERGENCY"
    LIMIT = "LIMIT"
    CURRENT_LIMIT = "CURRENT_LIMIT"
    HOMING = "HOMING"

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.instant_do = InstantDoCtrl(DEVICE_DESCRIPTION)
        self.instant_ao = InstantAoCtrl(DEVICE_DESCRIPTION)
        self.instant_ai = InstantAiCtrl(DEVICE_DESCRIPTION)
        self.instant_ao.channels[0].valueRange = ValueRange.V_Neg10To10
        

        self.current_position = self.load_current_position()
        self.current_angle = self.position_to_angle(self.current_position)  # 현재 각도 초기화

        self.filtered_velocity = 0  # 필터링된 각속도 초기화
        self.alpha = 0.5            # LPF 필터 계수 (0 < alpha < 1)
        self.offset = 0
        
        self.Limit_L, self.Limit_O, self.Limit_R = 0, 0, 0
        # PID 제어 변수
        self.kp = 0.1
        self.kd = 0.0
        self.previous_error = 0
        self.desired_angle = 0  # 목표 각도 초기화
        self.error = 0

        self.State = STATE.CALIBRATION

        ### HOMING Variable
        self.homing_direction = 1   # -1 or 1 
        self.homing_speed = 1.0     #[V] Voltage 출력



        # ROS 토픽 구독 및 퍼블리셔 설정
        self.pos_subscription = self.create_subscription(String, '/rcs/rail_refpos', self.pos_callback, 10)
        self.emg_subscription = self.create_subscription(Bool, '/rcs/rail_emg', self.emg_callback, 10)
        self.homing_subscription = self.create_subscription(Bool, '/rcs/rail_homing', self.homing_callback, 10)
        self.limit_subscription = self.create_subscription(Int16MultiArray, '/rcs/limit_sensor', self.limit_callback, 10)



        
        self.publisher_count = 0
        self.position_publisher = self.create_publisher(String, '/rcs/rail_actpos', 10)
        self.status_publisher = self.create_publisher(Bool, '/rcs/status', 10)
        # 시각화를 위한 퍼블리셔 설정
        # self.raw_velocity_publisher = self.create_publisher(Float32, '/motor/raw_velocity', 10)
        # self.filtered_velocity_publisher = self.create_publisher(Float32, '/motor/filtered_velocity', 10)
        self.angle_publisher = self.create_publisher(Float32, '/motor/angle', 10)
        self.desired_angle_publisher = self.create_publisher(Float32, '/motor/desired_angle', 10)
        self.current_publisher = self.create_publisher(Float32, 'motor/current', 10)
        
        self.previous_time = time.time()
        # ROS 타이머 설정
        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)


    def load_current_position(self):
        """ 파일에서 현재 위치를 불러오는 함수
        Returns:
            float: 현재 위치 (단위: 미터). 파일이 없거나 읽을 수 없으면 0 반환.
        """
        try:
            with open(POSITION_FILE, 'r') as f:
                position = float(f.read().strip())
                return position
        except FileNotFoundError:
            return 0
        except ValueError:
            return 0

    def save_current_position(self, position):
        """ 파일에 현재 위치를 저장하는 함수
        Args:
            position (float): 저장할 현재 위치 (단위: 미터)
        """
        with open(POSITION_FILE, 'w') as f:
            f.write(str(position))

    def calibration(self, calibration_time):
        """ 에러의 Offset을 얻기 위한 Calibration 함수
        Args:
            calibration_time (float): 보정 시간 (초)
        Returns:
            float: 보정된 오프셋 값
        """
        print("Calibration Start")
        print("Do not move the motor.")

        sampling_number = int(calibration_time / TIMER_PERIOD)
        sample_sum = 0
        for _ in range(sampling_number):
            raw_velocity = self.calculate_velocity(self.read_analog()[0])
            sample_sum += raw_velocity
            time.sleep(TIMER_PERIOD)
        return sample_sum / sampling_number

    def limit_callback(self, msg):
        """ 리밋 센서 데이터 수신 콜백 함수
        Args:
            msg (Int16MultiArray): 리밋 센서 데이터
        """
        self.Limit_L, self.Limit_O, self.Limit_R = msg.data[0], msg.data[1], msg.data[2] 
        
        if(self.Limit_O == 1):
            self.error = self.current_position
            print("CENTER DETECTED")

        if(self.Limit_L == 1 or self.Limit_R == 1):
            if(self.State != STATE.HOMING):
                self.State = STATE.LIMIT

    def homing_callback(self, msg):
        """ 보정 명령 수신 콜백 함수
        Args:
            msg (Bool): 보정 명령 여부
        """
        if msg.data:
            self.State = STATE.HOMING

    def emg_callback(self, msg):
        """ 비상 정지 명령 수신 콜백 함수
        Args:
            msg (Bool): 비상 정지 여부
        """
        if msg.data:
            self.State = STATE.EMERGENCY
        else:
            self.State = STATE.STEADYSTATE
    
    def pos_callback(self, msg):
        """ 목표 위치 수신 콜백 함수
        Args:
            msg (String): 목표 위치 (단위: 미터)
        """
        try:
            if(self.State == STATE.STEADYSTATE):
                self.State = STATE.PROCESS
                self.desired_position = float(msg.data)                             # unit : [m]
                self.desired_angle = self.position_to_angle(self.desired_position)  # unit : [degree]
        except:
            pass
    
    def write_digital(self, data):
        """ 디지털 출력 설정 함수
        Args:
            data (int): 출력할 데이터 (0 또는 1)
                        HIGH 신호를 보내면 모터 비상 정지
                        LOW 신호를 보내면 모터 정상 작동
        """
        self.instant_do.writeAny(0, 1, [data])
        
    def read_analog(self):
        """ 아날로그 입력을 읽는 함수
        Returns:
            list: 아날로그 데이터 (리스트 형태)
        """
        ai_channel = 0
        _, analog_data = self.instant_ai.readDataF64(ai_channel, 2)
        return analog_data
    
    def write_analog(self, voltage):
        """ 아날로그 출력을 설정하는 함수
        Args:
            voltage (float): 설정할 전압 (단위: V)
        """
        ao_channel = 0
        max_voltage = 10
        voltage = max(-max_voltage, min(max_voltage, voltage))  # 전압 제한
        self.instant_ao.writeAny(ao_channel, 1, None, [-voltage])

    def calculate_velocity(self, voltage):
        """ 주어진 전압에 따라 각속도를 계산하는 함수
        Args:
            voltage (float): 입력 전압 (단위: V)
        Returns:
            float: 계산된 각속도 (단위: deg/s)
        """
        return -voltage * 180  # 각속도 계산 (deg/s)

    def pid_control(self, pos_d, pos, dt):
        """ PID 제어 알고리즘을 사용하여 출력 값을 계산하는 함수
        Args:
            pos_d (float): 목표 위치
            pos (float): 현재 위치
            dt (float): 시간 간격 (초)

        Returns:
            float: PID 제어에 의해 계산된 출력 값
        """

        pos_error = pos_d - pos
        pos_derivative = (pos_error - self.previous_error) / dt
        output = self.kp * pos_error + self.kd * pos_derivative  # i 제어는 사용하지 않음
        self.previous_error = pos_error
        return output
    
    def angle_to_position(self, theta):
        pos = theta * 0.4523 / 720
        return pos
    
    def position_to_angle(self, pos):
        theta = pos * 720 / 0.4523
        return theta
    
    def control_loop(self):

        """ 제어 루프를 실행하는 함수 """
        
        self.publisher_count += 1
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        if(self.State == STATE.CALIBRATION):
            self.write_analog(0)
            self.offset = self.calibration(5)
            self.State = STATE.STEADYSTATE

        if(self.State == STATE.HOMING):
            if(self.Limit_L == 1):
                self.homing_direction = 1
            elif(self.Limit_R == 1):
                self.homing_direction = -1
            elif(self.Limit_O == 1):
                #   1. 정지
                #   2. 원점에서 부터 이동한 거리 계산
                #   3. 원점에서 부터 이동한 거리 만큼 돌아가기
                self.write_analog(0)
                time.sleep(0.2)
                self.State = STATE.STEADYSTATE

            self.write_analog(self.homing_speed * self.homing_direction)


        analog_input = self.read_analog()
        current = analog_input[1] * 5.9 / 4 # Analog to Current     -4V ~ 4V    :   -5.9A ~ 5.9A

        if(abs(current) > MAX_CURRENT):
            print(abs(current))
            self.State = STATE.CURRENT_LIMIT

        # Velocity를 받아와서 적분을 통해 angle을 구하는 과정
        raw_velocity = self.calculate_velocity(analog_input[0]) - self.offset
        # Low Pass Filter
        self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
        # 특정 값 사이의 속도를 무시하여 에러 적분 값이 누적되는 것을 막음
        VELOCITY_ERROR_BOUNDARY = 3
        if abs(self.filtered_velocity) < VELOCITY_ERROR_BOUNDARY:
            self.filtered_velocity = 0.0
        self.current_angle += self.filtered_velocity * dt  



        # ANGLE_ERROR_BOUNDARY 각도 안으로 진입하면 STEADYSTATE 상태
        ANGLE_ERROR_BOUNDARY = 5
        if(abs(self.current_angle - self.desired_angle) < ANGLE_ERROR_BOUNDARY):
            self.State = STATE.STEADYSTATE

        # 모터의 회전 각도로 레일의 위치 변환
        self.current_position = self.angle_to_position(self.current_angle)    # unit : [m]


        if(self.publisher_count > 10):
            self.position_publisher.publish(String(data = str(self.current_position)))
            self.save_current_position(self.current_position)
            self.publisher_count = 0

        """
            STATE에 따른 작업
            ---
            LIMIT           : 리밋센서에 인식된 상태로 프로세스 종료
            CURRENT_LIMIT   : 최대 부하 전류를 넘은 상태로 모터를 비상 정지 (충돌 감지)

            STEADYSTATE     : 정상상태로 다음 명령이 들어올 때까지 모터를 정지
            PROCESS         : 안정상태로 PID제어 수행
            EMERGENCY       : /rcs/rail_emg 토픽이 True로 들어온 상태로 정지
        """


        if(self.State == STATE.LIMIT):
            self.write_digital(1)   # 디지털 HIGH 신호를 보내면 모터 비상 정지
            self.write_analog(0)
            print("LIMIT SENSOR DETECTED")
            # raise Exception("LIMIT SENSOR DETECTED")

        elif(self.State == STATE.CURRENT_LIMIT):
            self.get_logger().info(f"[CURRENT_LIMIT] - CURRENT: {current}[A]")
            self.write_digital(1)   # 디지털 HIGH 신호를 보내면 모터 비상 정지
            self.write_analog(0)

        elif(self.State == STATE.STEADYSTATE):
            if(abs(self.error) > 0.01):
                self.current_position -= self.error
                self.current_angle = self.position_to_angle(self.current_position)
                print(f"error : {self.error}  영점 보정")
                self.error = 0
            # self.get_logger().info(f"[STEADYSTATE] - Pos: {self.current_position:.2f}")
            self.write_digital(0)
            self.write_analog(0)
            
            if(self.publisher_count > 10):
                self.status_publisher.publish(Bool(data = True))
            
        elif(self.State == STATE.PROCESS):
            print("PROCESS")
            self.write_digital(0)   # 디지털 LOW 신호를 보내면 모터 정상 작동
            self.status_publisher.publish(Bool(data = False))
            voltage_output = self.pid_control(self.desired_angle, self.current_angle, dt)
            voltage_output = max(-10, min(10, voltage_output)) # 출력 전압 제한
            self.get_logger().info(f"[PROCESS] - Pos: {self.current_position:.2f}, Desired_Pos: {self.desired_position}")
            self.write_analog(voltage_output)

        elif(self.State == STATE.EMERGENCY):
            self.get_logger().info(f"[EMERGENCY] - Pos: {self.current_position:.2f}")
            self.write_digital(1)   # 디지털 HIGH 신호를 보내면 모터 비상 정지
            self.write_analog(0)
        

        # 퍼블리시
        # self.raw_velocity_publisher.publish(Float32(data=raw_velocity))
        # self.filtered_velocity_publisher.publish(Float32(data=float(self.filtered_velocity)))
        self.angle_publisher.publish(Float32(data=float(self.current_angle)))
        self.desired_angle_publisher.publish(Float32(data=float(self.desired_angle)))
        self.current_publisher.publish(Float32(data=float(current)))

def main(args=None):
    """ ROS 2 노드를 초기화하고 실행하는 메인 함수 """
    try:
        rclpy.init(args=args)
        controller = MotorController()
        rclpy.spin(controller)

    except Exception as e:
        print("Paused:", e)

    finally:
        controller.write_digital(1) 
        controller.write_analog(0)
        time.sleep(0.1)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
