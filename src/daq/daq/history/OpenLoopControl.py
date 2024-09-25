import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.BDaqApi import BioFailed

# 초기 파라미터 설정
deviceDescription = "USB-4716,BID#0"
TORQUE_CONSTANT = 147  # 단위: mNm/A
TIMER_PERIOD = 0.001
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.instantAo = InstantAoCtrl(deviceDescription)
        self.instantAi = InstantAiCtrl(deviceDescription)
        self.instantAo.channels[0].valueRange = ValueRange.V_Neg10To10
        
        self.angle = 0  # 각도 초기화
        self.raw_velocity = []  # 각속도 기록
        self.data = []  # 데이터를 저장할 리스트
        self.filtered_velocity = 0  # 필터링된 각속도 초기화
        self.alpha = 0.7  # 필터 계수 (0 < alpha < 1)
        self.isFinish = False  # 작업 완료 여부
        self.offset = 0
        
        # PID 제어 변수
        self.Kp = 10.0   #9.0    20
        self.Ki = 0     
        self.Kd = 2   #1.0    3
        self.previous_error = 0

        self.calibration()
        # ROS 2 Subscriber 설정
        self.desired_angle = 0  # 목표 각도 초기화
        self.subscription = self.create_subscription(
            Int16,
            '/rail/position',
            self.listener_callback,
            10
        )
        # 필터링된 각속도를 발행하기 위한 퍼블리셔 설정
        self.raw_velocity_publisher = self.create_publisher(Float32, '/motor/raw_velocity', 10)
        self.filtered_velocity_publisher = self.create_publisher(Float32, '/motor/filtered_velocity', 10)
        self.angle_publisher = self.create_publisher(Float32, '/motor/angle', 10)
        self.desired_angle_publisher = self.create_publisher(Float32, '/motor/desired_angle', 10)
        
        self.msgIn = False

        self.previous_time = time.time()
        # 타이머 설정
        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)  # 0.1초마다 호출


    def listener_callback(self, msg):
        """
        ROS 2 메시지를 수신하는 콜백 함수입니다.
        
        Args:
            msg (Int16): ROS 2에서 수신한 Int16 타입의 메시지. 
                         이 메시지는 목표 각도를 포함하고 있습니다.
        
        Updates:
            self.desired_angle (int): 수신한 각도를 목표 각도로 설정합니다.
        """
        self.desired_angle = msg.data  # ROS에서 받은 목표 각도
        self.isFinish = False 
        self.msgIn = True

    def ReadAnalog(self):
        """
        아날로그 입력을 읽는 함수입니다.
        
        Returns:
            float: 읽어온 아날로그 데이터의 스케일된 값.
        """
        AIChannel = 0
        _, scaledData = self.instantAi.readDataF64(AIChannel, 1)
        return scaledData[0]
    
    def WriteAnalog(self, voltage):
        """
        아날로그 출력을 설정하는 함수입니다.
        
        Args:
            voltage (float): 설정할 전압 값. -10V에서 10V 사이의 값이어야 합니다.
        
        Raises:
            BioFailed: 데이터 쓰기 중 오류가 발생한 경우.
        """
        AOChannel = 0
        voltage = max(-10, min(10, voltage))  # 전압 제한
        ret = self.instantAo.writeAny(AOChannel, 1, None, [-voltage])
        if BioFailed(ret):
            self.get_logger().error("데이터 쓰기 중 오류 발생.")

    def CalculateVelocity(self, voltage):
        """
        주어진 전압에 따라 각속도를 계산하는 함수입니다.
        
        Args:
            voltage (float): 입력 전압 값.
        
        Returns:
            float: 계산된 각속도 (deg/s).
        """
        return -voltage * 180  # 각속도 계산 (deg/s)
    

    def CalculateVoltage(self, velocity):
        """
        주어진 각속도에 따라 전압을 계산하는 함수입니다.
        
        Args:
            float: 입력 각속도 (rpm).
        
        Returns:
            voltage (float): 계산된 전압 값.
        """
        return -velocity / 750  # 각속도 계산 (deg/s)

    def RpmToDegPerSec(self, rpm):
        deg_per_sec = rpm * (360 / 60)
        return deg_per_sec

    def control_loop(self):
        """
        제어 루프를 실행하는 함수입니다. 
        이 함수는 타이머에 의해 주기적으로 호출되어 모터의 제어를 수행합니다.
        """
        if(self.msgIn):
            maxrpm = 2500                                           # [rpm]
            angular_velocity = self.RpmToDegPerSec(maxrpm) / 25     # [deg/s] , 25 : 기어비
            voltage = self.CalculateVoltage(maxrpm)
            control_time = self.desired_angle / angular_velocity    # s = velocity_max * control_time
            self.WriteAnalog(voltage)
            time.sleep(control_time)
            self.WriteAnalog(0)
            self.msgIn = False


def main(args=None):
    """
    ROS 2 노드를 초기화하고 실행하는 메인 함수입니다.
    
    Args:
        args (list, optional): 커맨드라인 인자. 기본값은 None입니다.
    """
    try:
        rclpy.init(args=args)
        controller = MotorController()
        rclpy.spin(controller)

    except:
        print("Paused")
        controller.WriteAnalog(0)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
