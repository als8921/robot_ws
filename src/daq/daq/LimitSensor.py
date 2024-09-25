from CommonUtils import kbhit
from Automation.BDaq import *
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"

def ReadDigital():
    ret, data = instantDiCtrl.readAny(0, 1)
    if BioFailed(ret):
        return False

    limit = data[0] & 1
    print(limit)
    return limit

def Process():
    global instantDiCtrl
    instantDiCtrl = InstantDiCtrl(deviceDescription)

    while not kbhit():
        limit_data = ReadDigital()
        print(limit_data)


    instantDiCtrl.dispose()

if __name__ == '__main__':
    Process()
