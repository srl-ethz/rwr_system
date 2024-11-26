import time
from faive_system.src.hand_control.hand_controller import HandController

hc = HandController("/dev/ttyUSB0", calibration = True, auto_calibrate = False)
time.sleep(0.5)