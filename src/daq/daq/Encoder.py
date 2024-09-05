from CommonUtils import kbhit
from Automation.BDaq import *
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.BDaqApi import AdxEnumToString, BioFailed

deviceDescription = "USB-4716,BID#0"

def AdvInstantDI():
    ret = ErrorCode.Success
    instantDiCtrl = InstantDiCtrl(deviceDescription)
    print("Reading ports status is in progress, any key to quit!")
    count = 0
    last_encoderA, last_encoderB = 0, 0
    while not kbhit():
        ret, data = instantDiCtrl.readAny(0, 1)
        if BioFailed(ret):
            break

        encoderA = (data[0] >> 0) & 1
        encoderB = (data[0] >> 1) & 1

        if(last_encoderA == 0 and encoderA == 1):
            if(encoderB): count -= 1
            else: count += 1

        elif(last_encoderA == 1 and encoderA == 0):
            if(encoderB): count += 1
            else: count -= 1

        if(last_encoderB == 0 and encoderB == 1):
            if(encoderA): count += 1
            else: count -= 1

        elif(last_encoderB == 1 and encoderB == 0):
            if(encoderA): count -= 1
            else: count += 1

        print(f"angle : {count*6}")
        last_encoderA, last_encoderB = encoderA, encoderB
            
    print("\n DI output completed !")

    instantDiCtrl.dispose()

    if BioFailed(ret):
        enumStr = AdxEnumToString("ErrorCode", ret.value, 256)
        print("Some error occurred. And the last error code is %#x. [%s]" % (ret.value, enumStr))
    return 0


if __name__ == '__main__':
    AdvInstantDI()
