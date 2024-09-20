# import 부분
import time
import numpy as np
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.BDaqApi import BioFailed

# Initial Parameter
# cd /opt/advantech/tools && sudo ./dndev
deviceDescription = "USB-4716,BID#0"
instantDi = InstantDiCtrl(deviceDescription)
class Encoder:
    def ReadEncoder():
        """
        엔코더 A, B상 값을 받아와서 0, 1로 반환

        Return:
            (encoderA, encoderB)    엔코더 데이터
            False                   에러 발생
        """
        ret, data = instantDi.readAny(0, 1)
        if BioFailed(ret):
            return False

        encoderA = data[0] & 1
        encoderAa = (data[0] >> 1) & 1
        encoderB = (data[0] >> 2) & 1
        encoderBb = (data[0] >> 3) & 1

        A = encoderA - encoderAa
        if(A < 0): A = 0

        B = encoderB - encoderBb
        if(B < 0): B = 0
        return (A, B)

    def CountEncoder(encoderA, encoderB):
        """
        1상 1체배로 엔코더의 데이터로 CW 방향 1, CCW 방향 -1 반환
        
        Args:
            encoderA: 엔코더 A상 데이터 (0 or 1)
            encoderB: 엔코더 B상 데이터 (0 or 1)

        Return:
            1       CW
            -1      CCW
            0       None 
        """


        if(last_encoderA == 0 and encoderA == 1):
            return -1 if encoderB else 1

        elif(last_encoderA == 1 and encoderA == 0):
            return 1 if encoderB else -1

        if(last_encoderB == 0 and encoderB == 1):
            return 1 if encoderA else -1

        elif(last_encoderB == 1 and encoderB == 0):
            return -1 if encoderA else 1

        return 0

    def ConvertPulseToAngle(pulse_count):
        """
        펄스 수를 각도로 변환하는 함수

        Args:
            pulse_count: 현재까지의 펄스 수

        Return:
            encoder_angle: 변환된 각도
        """
        encoder_angle = pulse_count * 360 / 1000
        return encoder_angle

class MotorController:
    # 초기 세팅
    def __init__(self):
        self.instantAo = InstantAoCtrl(deviceDescription)
        self.instantAo.channels[0].valueRange = ValueRange.V_Neg10To10  #Voltage Output Range Setting

        self.angleDesire = 0
        self.angleCurrent = 0


        # PID 부분 Parameter
        self.Kp = 0.4
        self.Ki = 0
        self.Kd = 0
        self.previous_error = 0

        
    # 엔코더 값을 불러오는 부분



    # 아날로그 값을 보내주는 부분
    def WriteAnalog(self, voltage):
        AOChannel = 0
        
        if(voltage > 10): voltage = 10
        elif(voltage < -10): voltage = -10 

        ret = self.instantAo.writeAny(AOChannel, 1, None, [voltage])
        if BioFailed(ret):
            print("Error occurred during writing data.")


    # PID 제어 부분 Encoder -> 10To10V AnalogOutput
    def PIDControl(self, desirePosition, currentPosition, dt):
        pos_error = desirePosition - currentPosition
        pos_derivative = (pos_error - self.previous_error) / dt
        pos_integral = 0
        output =  self.Kp * pos_error + self.Kd * pos_derivative + self.Ki * pos_integral

        output = max(-10, min(10, output))

        self.previous_error = pos_error
        return output

    # 제어 명령을 받는 부분



    # 메인 제어 부분
    def main(self):

        global last_encoderA, last_encoderB
        last_encoderA, last_encoderB = 0, 0
        pulse_count = 0
        
        previous_time = time.time()
        try:
            while(True):
                encoder_data = Encoder.ReadEncoder()
                # 에러 발생시 Process 종료
                if not encoder_data:
                    break
                encoderA, encoderB = encoder_data

                pulse_count += Encoder.CountEncoder(encoderA, encoderB)
                last_encoderA, last_encoderB = encoderA, encoderB

                # 펄스를 각도로 변환
                encoder_angle = Encoder.ConvertPulseToAngle(pulse_count)


                """
                    PID 제어 부분
                """
                current_time = time.time()
                dt = current_time - previous_time
                previous_time = current_time

                desire_position = -10

                pid_output = self.PIDControl(desire_position, encoder_angle, dt)
                print(f"   current : {encoder_angle}\n   desire : {desire_position}\n   voltage : {pid_output}")
                print("=======================================")
                self.WriteAnalog(pid_output)
        except:
            pass

        finally:
            self.WriteAnalog(0)
            print("Paused")

if __name__ == "__main__":
    controller = MotorController()
    controller.main()