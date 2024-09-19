import numpy as np
from Automation.BDaq import *
from Automation.BDaq.InstantDoCtrl import InstantDoCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"

def AdvInstantDO():
    ret = ErrorCode.Success

    instantDo = InstantDoCtrl(deviceDescription)
    # Step 2: Write DO ports
    data = [0]
    while(1):
        data[0] = 0
        ret = instantDo.writeAny(0, 1, data)
        print("DO output completed!")

    instantDo.dispose()

    if BioFailed(ret):
        print("Some error occurred.")

if __name__ == "__main__":
    AdvInstantDO()
