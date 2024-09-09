import numpy as np
from Automation.BDaq import *
from Automation.BDaq.InstantAoCtrl import InstantAoCtrl
from Automation.BDaq.BDaqApi import BioFailed

deviceDescription = "USB-4716,BID#0"
AOChannel = 0
amplitude = 10 #Voltage Amplitude


def AnalogOut():
    instantAo = InstantAoCtrl(deviceDescription)
    instantAo.channels[0].valueRange = ValueRange.V_Neg10To10  #Voltage Output Range Setting

    i = 0
    while True:
        try :
            data = [amplitude * np.sin(i/50)]
            i+=1
            ret = instantAo.writeAny(AOChannel, 1, None, [data])
            if BioFailed(ret):
                print("Error occurred during writing data.")
                return
        except:
            ret = instantAo.writeAny(AOChannel, 1, None, [0])
            instantAo.dispose()
            print("System Paused")
            return
if __name__ == '__main__':
    AnalogOut()
