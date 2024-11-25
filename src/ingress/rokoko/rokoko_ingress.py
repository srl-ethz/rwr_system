#!/usr/bin/env python3

import socket
import json
import numpy as np
import threading
from scipy.spatial.transform import Rotation


MANO_KEYPOINTS_LIST = [
    "rightHand",
    "rightThumbProximal",
    "rightThumbMedial",
    "rightThumbDistal",
    "rightThumbTip",
    "rightIndexProximal",
    "rightIndexMedial",
    "rightIndexDistal",
    "rightIndexTip",
    "rightMiddleProximal",
    "rightMiddleMedial",
    "rightMiddleDistal",
    "rightMiddleTip",
    "rightRingProximal",
    "rightRingMedial",
    "rightRingDistal",
    "rightRingTip",
    "rightLittleProximal",
    "rightLittleMedial",
    "rightLittleDistal",
    "rightLittleTip",
]


class RokokoTracker:
    def __init__(self, ip="0.0.0.0", port=14043, use_coil=True):
        super(RokokoTracker, self).__init__()
        print("Starting Rokoko tracker")
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM  # Internet
        )  # "socket.SOCK_DGRAM" for UDP and "socket.SOCK_STREAM" for TCP
        sock.bind((ip, port))
        self.sock = sock

        self.keypoint_positions = None
        self.wrist_position = None
        self.time_stamp = None
        self.keypoints_lock = threading.Lock()
        self.wrist_lock = threading.Lock()

        self.keep_running = True
        self.thread = threading.Thread(target=self.read_rokoko_data)

        self.use_coil = use_coil

    def start(self):
        self.keep_running = True
        self.thread.start()

    def stop(self):
        self.keep_running = False
        self.thread.join()
        self.sock.close()

    def get_keypoint_positions(self):
        with self.keypoints_lock:
            if self.keypoint_positions is None:
                return None
            return self.keypoint_positions.copy(), self.time_stamp

    def set_keypoint_positions(self, keypoint_positions, timestamp):
        with self.keypoints_lock:
            self.keypoint_positions = keypoint_positions.copy()
            self.time_stamp = timestamp

    def set_wrist_pose(self, wrist_position, wrist_quat):
        with self.wrist_lock:
            self.wrist_position = wrist_position.copy()
            self.wrist_quat = wrist_quat.copy()

    def get_wrist_pose(self):
        with self.wrist_lock:
            if self.wrist_position is None or self.wrist_quat is None:
                KeyError("No wrist pose available")
                return None
            return self.wrist_position.copy(), self.wrist_quat.copy()

    def read_rokoko_data(self):
        while self.keep_running:
            received_data, addr = self.sock.recvfrom(
                1024 * 20
            )  # buffer size is 1024 bytes
            # print("Received message: %s" % received_data)
            try:
                data_dict = json.loads(
                    received_data.decode("utf-8")
                )  # Decode the received bytes and parse as a dictionary
            except:
                raise Exception("Error decoding received data")
            body_data = data_dict["scene"]["actors"][0]["body"]
            # get chest position
            keypoint_positions = []
            chest_position = np.array(
                [
                    body_data["chest"]["position"]["x"],
                    body_data["chest"]["position"]["y"],
                    body_data["chest"]["position"]["z"],
                ]
            )
            for bone_name in MANO_KEYPOINTS_LIST:
                local_position = np.array(
                    [
                        body_data[bone_name]["position"]["x"],
                        body_data[bone_name]["position"]["y"],
                        body_data[bone_name]["position"]["z"],
                    ]
                )
                # flip wrt chest position
                local_position = np.array(chest_position) - np.array(local_position)
                keypoint_positions.append(np.array(local_position))
            timestamp = data_dict["scene"]["timestamp"]
            self.set_keypoint_positions(np.array(keypoint_positions), timestamp)

            if self.use_coil:
                wrist_position = np.array(
                    [
                        body_data["rightHand"]["position"]["x"],
                        body_data["rightHand"]["position"]["y"],
                        body_data["rightHand"]["position"]["z"],
                    ]
                )
                # invert z axis since rokoko uses left handed coordinate system
                wrist_position[2] = -wrist_position[2]

                # invert sign on x, y, z since rokoko uses left handed coordinate system. ref: https://stackoverflow.com/a/28683097
                wrist_rot = Rotation.from_quat(
                    np.array(
                        [
                            body_data["rightHand"]["rotation"]["x"],
                            body_data["rightHand"]["rotation"]["y"],
                            body_data["rightHand"]["rotation"]["z"],
                            body_data["rightHand"]["rotation"]["w"],
                        ]
                    )
                )
                # invert sign on x, y, z since rokoko uses left handed coordinate system. ref: https://stackoverflow.com/a/28683097
                # wrist_quat = np.array([wrist_quat[0], wrist_quat[1], wrist_quat[2], wrist_quat[3]])
                # from quaternion to rotation matrix
                # wrist_rot = R.from_quat(wrist_quat).as_matrix()
                #  rotation matrix 180 degrees around z axis
                R_z_180 = Rotation.from_matrix(
                    np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                )
                wrist_rot = R_z_180 * wrist_rot
                wrist_quat = wrist_rot.as_quat()
                self.set_wrist_pose(wrist_position, wrist_quat)


if __name__ == "__main__":

    ip = "0.0.0.0"
    port = 14043
    use_coil = True
    tracker = RokokoTracker(ip=ip, port=port, use_coil=use_coil)
    tracker.start()

    def worker():
        while tracker.keep_running:
            print("keypoint positions", tracker.get_keypoint_positions())
            if use_coil:
                print("wrist", tracker.get_wrist_pose())

    t = threading.Thread(target=worker)
    input("Press any key to start, then press any key to stop")
    t.start()
    input("Press any key to stop")
    tracker.stop()
    t.join()
