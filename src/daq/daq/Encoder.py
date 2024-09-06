from CommonUtils import kbhit
from Automation.BDaq import *
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"

def ReadEncoder():
    """
    엔코더 A, B상 값을 받아와서 0, 1로 반환

    Return:
        (encoderA, encoderB)    엔코더 데이터
        False                   에러 발생
    """
    ret, data = instantDiCtrl.readAny(0, 1)
    if BioFailed(ret):
        return False

    encoderA = data[0] & 1
    encoderB = (data[0] >> 1) & 1

    return (encoderA, encoderB)

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
    if last_encoderA == 0 and encoderA == 1:
        return -1 if encoderB else 1
    return 0

def ConvertPulseToAngle(pulse_count):
    """
    펄스 수를 각도로 변환하는 함수

    Args:
        pulse_count: 현재까지의 펄스 수

    Return:
        encoder_angle: 변환된 각도
    """
    encoder_angle = pulse_count * 360 / 15
    return encoder_angle

def Process():
    global instantDiCtrl, last_encoderA, last_encoderB
    instantDiCtrl = InstantDiCtrl(deviceDescription)

    last_encoderA, last_encoderB = 0, 0
    pulse_count = 0

    while not kbhit():
        encoder_data = ReadEncoder()

        # 에러 발생시 Process 종료
        if not encoder_data:
            break
        encoderA, encoderB = encoder_data

        pulse_count += CountEncoder(encoderA, encoderB)
        last_encoderA, last_encoderB = encoderA, encoderB

        # 펄스를 각도로 변환
        encoder_angle = ConvertPulseToAngle(pulse_count)
        print(f"Angle : {encoder_angle}")

    instantDiCtrl.dispose()

if __name__ == '__main__':
    Process()
