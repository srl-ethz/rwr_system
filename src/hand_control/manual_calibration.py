import time
from faive_system.src.hand_control.hand_controller import HandController
import argparse

parser = argparse.ArgumentParser(description='Manual calibration script.')
parser.add_argument('--auto', default= True, help='Enable auto calibration')
args = parser.parse_args()

hc = HandController("/dev/ttyUSB0", calibration=not args.auto, auto_calibrate=args.auto)
time.sleep(0.5)