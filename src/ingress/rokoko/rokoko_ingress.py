#!/usr/bin/env python3

import socket
import json
import numpy as np
import threading
from scipy.spatial.transform import Rotation
from faive_system.src.common.enums import HandChirality

MANO_RIGHTHAND_KEYPOINTS_LIST = [
    "rightLowerArm",
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

MANO_LEFTHAND_KEYPOINTS_LIST = [
    "leftLowerArm",
    "leftHand",
    "leftThumbProximal",
    "leftThumbMedial",
    "leftThumbDistal",
    "leftThumbTip",
    "leftIndexProximal",
    "leftIndexMedial",
    "leftIndexDistal",
    "leftIndexTip",
    "leftMiddleProximal",
    "leftMiddleMedial",
    "leftMiddleDistal",
    "leftMiddleTip",
    "leftRingProximal",
    "leftRingMedial",
    "leftRingDistal",
    "leftRingTip",
    "leftLittleProximal",
    "leftLittleMedial",
    "leftLittleDistal",
    "leftLittleTip",
]


class RokokoTracker:
    def __init__(self, ip="0.0.0.0", port=14043, use_coil=True, enable_right=True, enable_left=True):
        super(RokokoTracker, self).__init__()
        print("Starting Rokoko tracker")
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM  # Internet
        )  # "socket.SOCK_DGRAM" for UDP and "socket.SOCK_STREAM" for TCP
        sock.bind((ip, port))
        sock.settimeout(0.25)
        self.sock = sock

        self.left_keypoint_positions, self.right_keypoint_positions = None, None
        self.left_wrist_position, self.right_wrist_position = None, None
        self.left_arm_position, self.right_arm_position = None, None
        self.time_stamp = None
        self.keypoints_lock = threading.Lock()
        self.arm_wrist_pose_lock = threading.Lock()

        self.keep_running = True
        self.thread = threading.Thread(target=self.read_rokoko_data)

        self.use_coil = use_coil
        self.enable_right = enable_right
        self.enable_left = enable_left

    def start(self):
        self.keep_running = True
        self.thread.start()

    def stop(self):
        self.keep_running = False
        self.thread.join()
        self.sock.close()

    def get_keypoint_positions(self):
        with self.keypoints_lock:
            if self.right_keypoint_positions is None or self.left_keypoint_positions is None:
                return None, None
            return (self.left_keypoint_positions.copy(), self.right_keypoint_positions.copy()), self.time_stamp

    def set_keypoint_positions(self, keypoint_positions, timestamp):
        with self.keypoints_lock:
            self.left_keypoint_positions = keypoint_positions[0].copy()
            self.right_keypoint_positions = keypoint_positions[1].copy()
            self.time_stamp = timestamp

    def set_wrist_poses(self, wrist_poses):
        with self.arm_wrist_pose_lock:
            left_wrist_pose, right_wrist_pose = wrist_poses

            self.left_wrist_position = left_wrist_pose[0].copy()
            self.left_wrist_quat = left_wrist_pose[1].copy()

            self.right_wrist_position = right_wrist_pose[0].copy()
            self.right_wrist_quat = right_wrist_pose[1].copy()

    def get_wrist_poses(self):
        with self.arm_wrist_pose_lock:
            if self.right_wrist_position is None or self.right_wrist_quat is None or self.left_wrist_position is None or self.left_wrist_quat is None:
                KeyError("No wrist pose available")
                return None
            return [(self.left_wrist_position.copy(), self.left_wrist_quat.copy()), (self.right_wrist_position.copy(), self.right_wrist_quat.copy())]

    def set_arm_poses(self, arm_poses):
        with self.arm_wrist_pose_lock:
            left_arm_pose, right_arm_pose = arm_poses

            self.left_arm_position = left_arm_pose[0].copy()
            self.left_arm_quat = left_arm_pose[1].copy()

            self.right_arm_position = right_arm_pose[0].copy()
            self.right_arm_quat = right_arm_pose[1].copy()

    def get_arm_poses(self):
        with self.arm_wrist_pose_lock:
            if self.right_arm_position is None or self.right_arm_quat is None or self.left_arm_position is None or self.left_arm_quat is None:
                KeyError("No arm pose available")
                return None
            return [(self.left_arm_position.copy(), self.left_arm_quat.copy()), (self.right_arm_position.copy(), self.right_arm_quat.copy())]

    def position_from_dict(self, dict):
        return np.array([dict["x"], dict["y"], dict["z"]])

    def quat_from_dict(self, dict):
        return np.array([dict["x"], dict["y"], dict["z"], dict["w"]])

    def read_rokoko_data(self):
        while self.keep_running:
            try:
                received_data, addr = self.sock.recvfrom(1024 * 20)
            except socket.timeout:
                continue
            # print("Received message: %s" % received_data)
            try:
                data_dict = json.loads(
                    received_data.decode("utf-8")
                )  # Decode the received bytes and parse as a dictionary
            except:
                raise Exception("Error decoding received data")
            body_data = data_dict["scene"]["actors"][0]["body"]
            # get chest position
            chest_position = self.position_from_dict(body_data["chest"]["position"])

            # Both chiralities are transformed identically: re-center each
            # keypoint relative to the chest. This produces honest left-hand
            # and right-hand keypoint frames. The previous code skipped this
            # flip on the left side to deliver pseudo-right-hand keypoints to
            # a right-hand-only URDF retargeter -- that hack is no longer
            # needed now that the retargeter has a left-hand URDF + scheme.
            righthand_keypoint_positions = []
            if self.enable_right:
                for bone_name in MANO_RIGHTHAND_KEYPOINTS_LIST:
                    local_position = self.position_from_dict(body_data[bone_name]["position"])
                    local_position = np.array(chest_position) - np.array(local_position)
                    righthand_keypoint_positions.append(np.array(local_position))

            lefthand_keypoint_positions = []
            if self.enable_left:
                for bone_name in MANO_LEFTHAND_KEYPOINTS_LIST:
                    local_position = self.position_from_dict(body_data[bone_name]["position"])
                    local_position = np.array(chest_position) - np.array(local_position)
                    lefthand_keypoint_positions.append(np.array(local_position))

            timestamp = data_dict["scene"]["timestamp"]
            assert not self.enable_left or len(lefthand_keypoint_positions) == 22, "Need 22 left keypoints including forearm and wrist"
            assert not self.enable_right or len(righthand_keypoint_positions) == 22, "Need 22 right keypoints including forearm and wrist"
            self.set_keypoint_positions((np.array(lefthand_keypoint_positions), np.array(righthand_keypoint_positions)), timestamp)

            # get wrist position
            if self.use_coil:
                wrist_poses, arm_poses = [], []
                for chirality in HandChirality:
                    if not ((chirality == "left" and self.enable_left) or (chirality == "right" and self.enable_right)):
                        # Skip processing if hand is disabled, just add dummy poses
                        wrist_poses.append((np.zeros(3), np.array([0, 0, 0, 1])))
                        arm_poses.append((np.zeros(3), np.array([0, 0, 0, 1])))
                        continue
                        
                    hand_name = chirality + "Hand"
                    wrist_position = self.position_from_dict(body_data[hand_name]["position"])
                    # invert z axis since rokoko uses left handed coordinate system
                    wrist_position[2] = -wrist_position[2]
                    wrist_rot = Rotation.from_quat(self.quat_from_dict(body_data[hand_name]["rotation"]))

                    #  rotation matrix 180 degrees around z axis
                    R_z_180 = Rotation.from_matrix(
                        np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                    )
                    wrist_rot = R_z_180 * wrist_rot
                    wrist_quat = wrist_rot.as_quat()
                    wrist_poses.append((wrist_position, wrist_quat))


                    arm_name = chirality + "LowerArm"
                    arm_position = self.position_from_dict(body_data[arm_name]["position"])
                    arm_position[2] = -arm_position[2]


                    # invert sign on x, y, z since rokoko uses left handed coordinate system. ref: https://stackoverflow.com/a/28683097
                    arm_rot = Rotation.from_quat(self.quat_from_dict(body_data[arm_name]["rotation"]))

                    #  rotation matrix 180 degrees around z axis
                    R_z_180 = Rotation.from_matrix(
                        np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                    )
                    arm_rot = R_z_180 * arm_rot
                    arm_quat = arm_rot.as_quat()
                    arm_poses.append((arm_position, arm_quat))
                self.set_wrist_poses(wrist_poses)
                self.set_arm_poses(arm_poses)


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
                print("wrist", tracker.get_wrist_poses())

    t = threading.Thread(target=worker)
    input("Press any key to start, then press any key to stop")
    t.start()
    input("Press any key to stop")
    tracker.stop()
    t.join()
