import time
from faive_system.src.hand_control.hand_controller import HandController

hc = HandController("/dev/ttyUSB0", calibration = True)
hc.connect_to_dynamixels()
time.sleep(3.0)