from faive_system.src.ingress.rokoko.rokoko_ingress import RokokoTracker
import threading
import json
import datetime
import argparse

def run_recorder():
    ip = "0.0.0.0"
    port = 14043
    use_coil = False
    tracker = RokokoTracker(ip=ip, port=port, use_coil=use_coil)
    tracker.start()

    datetime_str = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    handposes_filename = f"handposes_{datetime_str}.json"

    user_input = ''
    handpose_list = []
    while user_input != "q":
        user_input = input("Press any key to record keypoints, q to stop")
        handposes, timestamp = tracker.get_keypoint_positions()
        left_handpose, right_handpose = handposes
        handpose_list.append([left_handpose.tolist(), right_handpose.tolist(), timestamp])

        with open(handposes_filename, "w") as f:
            json.dump(handpose_list, f)

    tracker.stop()


if __name__ == "__main__":
    run_recorder()