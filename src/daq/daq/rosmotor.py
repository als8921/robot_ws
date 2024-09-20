# import 부분
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.BDaqApi import BioFailed

# Initial Parameter
# cd /opt/advantech/tools && sudo ./dndev
deviceDescription = "USB-4716,BID#0"

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        # self.instantAo = InstantAoCtrl(deviceDescription)
        # self.instantAi = InstantAiCtrl(deviceDescription)
        # self.instantAo.channels[0].valueRange = ValueRange.V_Neg10To10  # Voltage Output Range Setting
        self.angle = 0  # 각도 초기화
        self.filtered_velocity = 0  # 필터링된 각속도 초기화
        self.alpha = 0.6  # 필터 계수 (0 < alpha < 1)

        # PID 부분 Parameter
        self.Kp = 0.1
        self.Ki = 0
        self.Kd = 0.02
        self.previous_error = 0

        # ROS2 Subscriber 설정
        self.desire_position = 0.0
        self.subscription = self.create_subscription(
            Float32,
            'desired_position',
            self.desired_position_callback,
            10
        )

    def desired_position_callback(self, msg):
        print(msg.data)
        self.desire_position = msg.data

    # 아날로그 값을 받아오는 부분
    def ReadAnalog(self):
        AIChannel = 0
        _, scaledData = self.instantAi.readDataF64(AIChannel, 1)
        return scaledData[0]

    def CalculateVelocity(self, voltage):
        """
            V                     inner velocity   outer velocity
           -4V  :   -3000   rpm    -18000 deg/s       720 deg/s
            0V  :       0   rpm         0 deg/s         0 deg/s
            4V  :    3000   rpm     18000 deg/s      -720 deg/s

            Gear Ratio = 1 : 25
        """
        velocity = -voltage * 180  # deg/s
        return velocity

    # 아날로그 값을 보내주는 부분
    def WriteAnalog(self, voltage):
        AOChannel = 0
        
        if voltage > 10:
            voltage = 10
        elif voltage < -10:
            voltage = -10 

        ret = self.instantAo.writeAny(AOChannel, 1, None, [-voltage])
        if BioFailed(ret):
            print("Error occurred during writing data.")

    # PID 제어 부분 -> 10To10V AnalogOutput
    def PIDControl(self, desirePosition, currentPosition, dt):
        pos_error = desirePosition - currentPosition
        pos_derivative = (pos_error - self.previous_error) / dt
        pos_integral = 0
        output = self.Kp * pos_error + self.Kd * pos_derivative + self.Ki * pos_integral

        output = max(-10, min(10, output))

        self.previous_error = pos_error
        return output

    # 메인 제어 부분
    def main(self):
        try:
            previous_time = time.time()
            while rclpy.ok():
                rclpy.spin_once(self)
                current_time = time.time()
                dt = current_time - previous_time
                previous_time = current_time

                # raw_velocity = self.CalculateVelocity(self.ReadAnalog())
                
                # Low Pass Filter
                # self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
                
                # 각도 적분
                MIN_ANGULAR_VELOCITY = 2  # 정지시 노이즈를 해결하기 위해 적분하는 최소 속도 설정
                if abs(self.filtered_velocity) > MIN_ANGULAR_VELOCITY:
                    self.angle += self.filtered_velocity * dt  # 각도 업데이트

                ErrorBoundary = 5  # 목표 값 도달 범위 설정
                if abs(self.desire_position - self.angle) < ErrorBoundary:
                    pass
                    # self.WriteAnalog(0)
                    # print("Done")
                else:
                    pid_output = self.PIDControl(self.desire_position, self.angle, dt)
                    # self.WriteAnalog(pid_output)    
                    print(f"angular velocity (filtered): {self.filtered_velocity:.2f} [deg/s], angle: {self.angle:.2f} [degrees], desired position: {self.desire_position:.2f}")

        except Exception as e:
            print(f"Error occurred: {e}")
        finally:
            # self.WriteAnalog(0)
            print("Paused")

if __name__ == "__main__":
    rclpy.init()
    controller = MotorController()
    controller.main()
    rclpy.shutdown()
