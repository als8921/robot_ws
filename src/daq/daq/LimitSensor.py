from CommonUtils import kbhit
from Automation.BDaq import *
from Automation.BDaq.InstantDiCtrl import InstantDiCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"

def ReadDigital():
    ret, data = instantDiCtrl.readAny(0, 1)
    if BioFailed(ret):
        return False

    limit_data = [0] * 3
    limit_data[0] = data[0] & 1
    limit_data[1] = (data[0] >> 1) & 1
    limit_data[2] = (data[0] >> 2) & 1
    return limit_data

def Process():
    global instantDiCtrl
    instantDiCtrl = InstantDiCtrl(deviceDescription)
    i = 0
    while not kbhit():
        i += 1
        left_limit, origin_limit, right_limit = ReadDigital()
        if(left_limit & right_limit & origin_limit):
            pass
        else:
            print("LEFT ===== ORIGIN ===== RIGHT")
            print(f"   {left_limit}          {origin_limit}         {right_limit} ")

            
            if(left_limit == 1):
                print("LEFT DETECTED", i)
            if(right_limit == 1):
                print("RIGHT DETECTED", i)
            if(origin_limit == 1):
                print("ORIGIN", i)

    instantDiCtrl.dispose()

if __name__ == '__main__':
    Process()
