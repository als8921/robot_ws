from Automation.BDaq import *
from Automation.BDaq.InstantAiCtrl import InstantAiCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"

AIChannel = 0

def AdvInstantAI():
    ret = ErrorCode.Success

    instanceAiObj = InstantAiCtrl(deviceDescription)
    
    print("Acquisition is in progress, any key to quit!")
    while(True):
        ret, scaledData = instanceAiObj.readDataF64(AIChannel, 1)
        print(scaledData[0])

        if BioFailed(ret):
            break



    instanceAiObj.dispose()



if __name__ == '__main__':
    AdvInstantAI()
