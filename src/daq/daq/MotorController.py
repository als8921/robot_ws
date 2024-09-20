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
        self.instantAo.channels[0].valueRange = ValueRange.V_Neg10To10  #Voltage Output Range Setting
    
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
        velocity = voltage * 180 # deg/s
        return velocity


    # 아날로그 값을 보내주는 부분
    def WriteAnalog(self, voltage):
        AOChannel = 0
        
        if(voltage > 10): voltage = 10
        elif(voltage < -10): voltage = -10 

        ret = self.instantAo.writeAny(AOChannel, 1, None, [voltage])
        if BioFailed(ret):
            print("Error occurred during writing data.")


    # 메인 제어 부분
    def main(self):
        
        try:
            while(True):
                self.WriteAnalog(4)
                print(f"angular velocity : {self.CalculateVelocity(self.ReadAnalog())} [deg/s]")
        except:
            pass

        finally:
            self.WriteAnalog(0)
            print("Paused")


if __name__ == "__main__":
    controller = MotorController()
    controller.main()