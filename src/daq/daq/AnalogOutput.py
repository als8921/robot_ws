import numpy as np
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"
AOChannel = 0
amplitude = 2.5


def AnalogOut():
    instantAo = InstantAoCtrl(deviceDescription)
    i = 0
    while True:
        data = [amplitude + amplitude * np.sin(i/20000)]
        i+=1
        ret = instantAo.writeAny(AOChannel, 1, None, data)
        if BioFailed(ret):
            print("Error occurred during writing data.")
            return

if __name__ == '__main__':
    AnalogOut()
