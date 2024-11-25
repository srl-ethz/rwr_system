import time
from faive_system.src.hand_control.hand_controller import HandController

hc = HandController("/dev/ttyUSB0", calibration = True)
time.sleep(3.0)