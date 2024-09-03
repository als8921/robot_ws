import PythonLibMightyZap_PC as MightyZap
import time

Actuator_ID = 3

pos = 1000
MightyZap.OpenMightyZap('/dev/ttyUSB0',57600)
MightyZap.SetErrorIndicatorEnable(Actuator_ID, 0)
print(MightyZap.GetErrorIndicatorEnable(Actuator_ID))
time.sleep(1)
MightyZap.SetErrorIndicatorEnable(Actuator_ID, 1)
print(MightyZap.GetErrorIndicatorEnable(Actuator_ID))
time.sleep(1)
MightyZap.GoalPosition(Actuator_ID, pos)

while(abs(MightyZap.PresentPosition(Actuator_ID) - pos) > 10):
    print(MightyZap.PresentPosition(Actuator_ID))

print(MightyZap.PresentPosition(Actuator_ID))

MightyZap.ForceEnable(Actuator_ID, 0)

MightyZap.CloseMightyZap()