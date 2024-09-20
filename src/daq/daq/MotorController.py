# import 부분
import time
import numpy as np
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.BDaqApi import BioFailed

# Initial Parameter
# cd /opt/advantech/tools && sudo ./dndev
deviceDescription = "USB-4716,BID#0"

class MotorController:
    # 초기 세팅
    def __init__(self):
        self.instantAo = InstantAoCtrl(deviceDescription)
        self.instantAi = InstantAiCtrl(deviceDescription)
        self.instantAo.channels[0].valueRange = ValueRange.V_Neg10To10  # Voltage Output Range Setting
        self.angle = 0  # 각도 초기화
        self.filtered_velocity = 0  # 필터링된 각속도 초기화
        self.alpha = 0.6 # 필터 계수 (0 < alpha < 1)

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

    # 메인 제어 부분
    def main(self):
        try:
            previous_time = time.time()
            while True:
                current_time = time.time()
                dt = current_time - previous_time
                previous_time = current_time

                self.WriteAnalog(2)
                raw_velocity = self.CalculateVelocity(self.ReadAnalog())
                
                # Low Pass Filter
                self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
                
                # 각도 적분
                if(abs(self.filtered_velocity) > 2):
                    self.angle += self.filtered_velocity * dt  # 각도 업데이트

                print(f"angular velocity (filtered): {self.filtered_velocity:.2f} [deg/s], angle: {self.angle:.2f} [degrees]")
        except Exception as e:
            print(f"Error occurred: {e}")
        finally:
            self.WriteAnalog(0)
            print("Paused")

if __name__ == "__main__":
    controller = MotorController()
    controller.main()
