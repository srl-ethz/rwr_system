# dependencies for retargeter
import time
import torch
import numpy as np
from torch.nn.functional import normalize
import os
import pytorch_kinematics as pk
from .utils import retarget_utils
from typing import Union
import yaml
from scipy.spatial.transform import Rotation
from collections import defaultdict
import torch

def transform_from_anchor(
    chain_transforms,
    target_frame: str,
    anchor_frame: str, # or None
):
    """
    Get the chain transforms with respect to the anchor frame
    """
    transform_target = chain_transforms[target_frame]
    if anchor_frame is None:
        return transform_target
    else:
        transform_anchor = chain_transforms[anchor_frame]
        # return transform_target.compose(transform_anchor.inverse()) 
        return transform_anchor.inverse().compose(transform_target)

class Retargeter:
    """
    Please note that the computed joint angles of the rolling joints are only half of the two joints combined.
    hand_scheme either a string of the yaml path or a dictionary of the hand scheme
    mano_adjustments is a dictionary of the adjustments to the mano joints.
        keys: "thumb", "index", "middle", "ring", "pinky"
        value is a dictionary with the following keys:
            translation: (3,) translation vector in palm frame
            rotation: (3) x,y,z angles in palm frame, around the finger base
            scale: (3,) scale factors in finger_base frame
    retargeter_cfg is a dictionary of the retargeter algorithm. Including the following options:
        lr: learning rate
        loss_func_cfg: dictionary of loss function configuration 
            {loss_f_name: loss_f_weight} 
            or {loss_f_name: {weight: loss_f_weight, kwargs: loss_f_args}} 
    """

    def __init__(
        self,
        urdf_filepath: str = None,
        mjcf_filepath: str = None,
        sdf_filepath: str = None,
        urdf_string: str = None,
        hand_scheme:  Union[str, dict] = None,
        mano_adjustments: Union[str, dict] = None,
        retargeter_cfg: Union[str, dict] = None,
        optimizer: str = "RMSprop",
        device: str = None
    ) -> None:
        assert (
            int(urdf_filepath is not None)
            + int(mjcf_filepath is not None)
            + int(sdf_filepath is not None)
            + int(urdf_string is not None)
        ) == 1, "Exactly one of urdf_filepath, urdf_string, mjcf_filepath, or sdf_filepath should be provided"

        if hand_scheme == None:
            print("No hand scheme provided. Assuming Franka gripper")
            self.use_franka_gripper = True
        else:
            self.use_franka_gripper = False
            if isinstance(hand_scheme, dict):
                pass
            
            elif isinstance(hand_scheme, str):
                with open(hand_scheme, "r") as f:
                    print(f"Loading hand scheme from {hand_scheme}")
                    hand_scheme = yaml.safe_load(f)
            else:
                raise ValueError("hand_scheme should be a string or dictionary")

        if mano_adjustments is None:
            self._mano_adjustments = {}
        elif isinstance(mano_adjustments, dict):
            self._mano_adjustments = mano_adjustments
        elif isinstance(mano_adjustments, str):
            with open(mano_adjustments, "r") as f:
                    self._mano_adjustments = yaml.safe_load(f)
                    
        self.model_center, self.model_rotation = None, None
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        if hand_scheme is not None:
            GC_TENDONS = hand_scheme["gc_tendons"]
            FINGER_TO_TIP = hand_scheme["finger_to_tip"]
            FINGER_TO_BASE = hand_scheme["finger_to_base"]
            GC_LIMITS_LOWER = hand_scheme["gc_limits_lower"]
            GC_LIMITS_UPPER = hand_scheme["gc_limits_upper"]
            self.wrist_name = hand_scheme["wrist_name"]
            self.anchor_name = hand_scheme.get("anchor_name", None)
            self.forearm_and_wrist = hand_scheme.get("forearm_and_wrist", None) # used by loss_keyvector_forearm_and_wrist

            if retargeter_cfg is None:
                self._retargeter_cfg = {
                    "lr": 2.5,
                    "loss_func_cfg": {
                        "keyvector_fingers2palm": 1.0
                    }
                }
            elif isinstance(retargeter_cfg, dict):
                self._retargeter_cfg = retargeter_cfg
            elif isinstance(retargeter_cfg, str):
                with open(retargeter_cfg, "r") as f:
                    self._retargeter_cfg = yaml.safe_load(f)

            self.lr = self._retargeter_cfg["lr"]

            self.target_angles = None


            self.gc_limits_lower = GC_LIMITS_LOWER
            self.gc_limits_upper = GC_LIMITS_UPPER
            self.finger_to_tip = FINGER_TO_TIP
            self.finger_to_base = FINGER_TO_BASE

            prev_cwd = os.getcwd()
            if urdf_string is not None:
                # In-memory URDF — meshes use package:// URIs that resolve
                # via the ROS package system, so no working-directory
                # contortions are needed for kinematics.
                self.chain = pk.build_chain_from_urdf(urdf_string).to(
                    device=self.device
                )
            else:
                model_path = (
                    urdf_filepath
                    if urdf_filepath is not None
                    else mjcf_filepath if mjcf_filepath is not None else sdf_filepath
                )
                model_dir_path = os.path.dirname(model_path)
                os.chdir(model_dir_path)
                if urdf_filepath is not None:
                    self.chain = pk.build_chain_from_urdf(open(urdf_filepath).read()).to(
                        device=self.device
                    )
                elif mjcf_filepath is not None:
                    self.chain = pk.build_chain_from_mjcf(open(mjcf_filepath).read()).to(
                        device=self.device
                    )
                elif sdf_filepath is not None:
                    self.chain = pk.build_chain_from_sdf(open(sdf_filepath).read()).to(
                        device=self.device
                    )
                os.chdir(prev_cwd)

            ## This part builds the `joint_map` (n_joints, n_tendons) which is jacobian matrix.
            ## each tendon is a group of coupled joints that are driven by a single motor
            ## The rolling contact joints are modeled as a pair of joint and virtual joint
            ## The virtual joints are identified by the suffix "_virt"
            ## So, the output of the virtual joint will be the sum of the joint and its virtual counterpart, i.e. twice 
            joint_parameter_names = self.chain.get_joint_parameter_names()
            gc_tendons = GC_TENDONS
            self.n_joints = self.chain.n_joints
            self.n_tendons = len(
                GC_TENDONS
            )  # each tendon can be understand as the tendon drive by a motor individually
            self.joint_map = torch.zeros(self.n_joints, self.n_tendons).to(device)
            self.finger_to_tip = FINGER_TO_TIP
            self.tendon_names = []
            joint_names_check = []
            for i, (name, tendons) in enumerate(gc_tendons.items()):
                virtual_joint_weight = 0.5 if name.endswith("_virt") else 1.0
                self.joint_map[joint_parameter_names.index(name), i] = virtual_joint_weight
                self.tendon_names.append(name)
                joint_names_check.append(name)
                for tendon, weight in tendons.items():
                    self.joint_map[joint_parameter_names.index(tendon), i] = (
                        weight * virtual_joint_weight
                    )
                    joint_names_check.append(tendon)
            assert set(joint_names_check) == set(
                joint_parameter_names
            ), "Joint names mismatch, please double check hand_scheme"

            self.gc_joints = torch.ones(self.n_tendons).to(self.device) * 15.0
            self.gc_joints.requires_grad_()

            # Initialize optimizer based on the parameter
            if optimizer.lower() == "adam":
                self.opt = torch.optim.Adam([self.gc_joints], lr=self.lr)
            elif optimizer.lower() == "rmsprop":
                self.opt = torch.optim.RMSprop([self.gc_joints], lr=self.lr)
            else:
                raise ValueError(f"Unsupported optimizer: {optimizer}. Use one of: Adam, RMSprop")

            self.root = torch.zeros(1, 3).to(self.device)
            self.frames_we_care_about = None

            self.sanity_check()
            _chain_transforms = self.chain.forward_kinematics(
                torch.zeros(self.chain.n_joints, device=self.chain.device)
            )


            self.model_center, self.model_rotation = (
                retarget_utils.get_hand_center_and_rotation(
                    thumb_base=transform_from_anchor(_chain_transforms, self.finger_to_base["thumb"], self.anchor_name).transform_points(self.root),
                    index_base=transform_from_anchor(_chain_transforms, self.finger_to_base["index"], self.anchor_name).transform_points(self.root),
                    middle_base=transform_from_anchor(_chain_transforms, self.finger_to_base["middle"], self.anchor_name).transform_points(self.root),
                    ring_base=transform_from_anchor(_chain_transforms, self.finger_to_base["ring"], self.anchor_name).transform_points(self.root),
                    pinky_base=transform_from_anchor(_chain_transforms, self.finger_to_base["pinky"], self.anchor_name).transform_points(self.root),
                    wrist=transform_from_anchor(_chain_transforms, self.wrist_name, self.anchor_name).transform_points(self.root),
                )
            )
            self.model_center = self.model_center.cpu().numpy()
            self.model_rotation = self.model_rotation.cpu().numpy()
            assert np.allclose(
                (self.model_rotation @ self.model_rotation.T), (np.eye(3)), atol=1e-6
            ), "Model rotation matrix is not orthogonal"


            self.debug_timestamps = []
            self.debug_log_prefix = ""
            self.debug_event_counters = defaultdict(int)

            self._loss_function = self.build_loss_function(self._retargeter_cfg["loss_func_cfg"])


    def sanity_check(self):
        """
        Check if the chain and scheme configuration is correct
        """

        ## Check the tip and base frames exist
        for finger, tip in self.finger_to_tip.items():
            assert (
                tip in self.chain.get_link_names()
            ), f"Tip frame {tip} not found in the chain"
        for finger, base in self.finger_to_base.items():
            assert (
                base in self.chain.get_link_names()
            ), f"Base frame {base} not found in the chain"

    @property
    def mano_adjustments(self):
        return self._mano_adjustments
    @mano_adjustments.setter
    def mano_adjustments(self, adjustments):
        self._mano_adjustments = adjustments

    @property
    def retargeter_cfg(self):
        return self._retargeter_cfg
    @retargeter_cfg.setter
    def retargeter_cfg(self, cfg):
        self._retargeter_cfg = cfg
        self.lr = cfg["lr"]
        self._loss_function = self.build_loss_function(cfg["loss_func_cfg"])

    def retarget_finger_mano_joints(
        self,
        joints: np.array,
        warm: bool = True,
        opt_steps: int = 2,
        dynamic_keyvector_scaling: bool = False,
    ):
        """
        Process the MANO joints and update the finger joint angles
        joints: (22, 3)
        Over the 22 dims:
        0: forearm
        1: wrist
        2-5: thumb (from hand base)
        6-9: index
        10-13: middle
        14-17: ring
        18-21: pinky
        """

        # print(f"Retargeting: Warm: {warm} Opt steps: {opt_steps}")
        if self.frames_we_care_about is None:
            frames_names = []
            frames_names.append(self.finger_to_base["thumb"])
            frames_names.append(self.finger_to_base["pinky"])
            for finger, finger_tip in self.finger_to_tip.items():
                frames_names.append(finger_tip)
            if self.forearm_and_wrist is not None:
                for finger, finger_tip in self.forearm_and_wrist.items():
                    frames_names.append(finger_tip)
            self.frames_we_care_about = self.chain.get_frame_indices(*frames_names)

        start_time = time.time()
        if not warm:
            self.gc_joints = torch.ones(self.n_joints).to(self.device) * 15.0
            self.gc_joints.requires_grad_()

        assert joints.shape == (
            22,
            3,
        ), "The shape of the mano joints array should be (22, 3) including the forearm"

        mano_points = torch.from_numpy(joints).to(self.device)
        # norms_mano = {k: torch.norm(v) for k, v in keyvectors_mano.items()}
        # print(f"keyvectors_mano: {norms_mano}")
        for step in range(opt_steps):
            loss = self._loss_function(self.gc_joints, mano_points)
            self.opt.zero_grad()
            loss.backward()
            self.opt.step()
            with torch.no_grad():
                self.gc_joints[:] = torch.clamp(
                    self.gc_joints,
                    torch.tensor(self.gc_limits_lower).to(self.device),
                    torch.tensor(self.gc_limits_upper).to(self.device),
                )

        finger_joint_angles = self.gc_joints.detach().cpu().numpy()

        retarget_time_ms = (time.time() - start_time) * 1e3
        warn_threshold = 40  # ms
        if retarget_time_ms > warn_threshold:
            print(f"Retarget time: {retarget_time_ms} ms")
            print(f"Is CUDA MPS daemon running? If not, run 'sudo nvidia-cuda-mps-control -d")

        return finger_joint_angles

    def adjust_mano_fingers(self, joints):

        # Assuming mano_adjustments is accessible within the class
        mano_adjustments = self._mano_adjustments

        if mano_adjustments.get("all") is not None:
            # the adjustments for all points
            translation = mano_adjustments["all"].get("translation", np.zeros(3))  # (3,)
            rotation_angles = mano_adjustments["all"].get("rotation", np.zeros(3))  # (3,)
            scale = mano_adjustments["all"].get("scale", np.ones(3))  # (3,)
            # Apply the adjustments to all joints
            joints = joints * scale + translation
            rot = Rotation.from_euler("xyz", rotation_angles, degrees=False)
            R_matrix = rot.as_matrix()  # Rotation matrix (3,3)
            joints = joints @ R_matrix.T  # Apply rotation
        

        # Get the joints per finger
        joints_dict = retarget_utils.get_mano_joints_dict(joints)

        # Initialize adjusted joints dictionary
        adjusted_joints_dict = {}

        # Process each finger
        for finger in ["thumb", "index", "middle", "ring", "pinky"]:
            # Original joints for the finger
            finger_joints = joints_dict[finger]  # Shape: (n_joints, 3)

            if  mano_adjustments.get(finger) is None:
                adjusted_joints_dict[finger] = finger_joints
                continue
            # Adjustments for the finger
            adjustments = mano_adjustments[finger]
            translation = adjustments.get("translation", np.zeros(3))  # (3,)
            rotation_angles = adjustments.get("rotation", np.zeros(3))  # (3,)
            scale = adjustments.get("scale", np.ones(3))  # (3,)

            # Scaling in the finger base frame
            x_base = finger_joints[0]  # Base joint position (3,)
            x_local = finger_joints - x_base  # Local coordinates (n_joints, 3)
            x_local_scaled = x_local * scale  # Apply scaling

            # Rotation around base joint in palm frame
            rot = Rotation.from_euler("xyz", rotation_angles, degrees=False)
            R_matrix = rot.as_matrix()  # Rotation matrix (3,3)
            x_local_rotated = x_local_scaled @ R_matrix.T  # Apply rotation
            finger_joints_rotated = x_base + x_local_rotated  # Rotated positions

            # Translation in palm frame
            finger_joints_adjusted = finger_joints_rotated + translation  # Adjusted positions

            # Store adjusted joints
            adjusted_joints_dict[finger] = finger_joints_adjusted

        # Keep the wrist as is
        adjusted_joints_dict["wrist"] = joints_dict["wrist"]
        adjusted_joints_dict["forearm"] = joints_dict["forearm"]

        # Concatenate adjusted joints
        joints = np.concatenate(
            [
                adjusted_joints_dict["forearm"].reshape(1, -1),
                adjusted_joints_dict["wrist"].reshape(1, -1),
                adjusted_joints_dict["thumb"],
                adjusted_joints_dict["index"],
                adjusted_joints_dict["middle"],
                adjusted_joints_dict["ring"],
                adjusted_joints_dict["pinky"],
            ],
            axis=0,
        )

        return joints

    def retarget_franka_gripper_joints(self, joints):
        '''
        inputs : mano joints
        return: width of the gripper
        
        '''
        thumb_fingertip = joints[4]
        index_fingertip = joints[8]
        
        # get the distance between thumb and index fingertip
        dist = np.linalg.norm(thumb_fingertip - index_fingertip)
        return np.rad2deg(dist)
    
    def add_debug_timestamp(self, event_id):
        timestamp = time.monotonic()
        event_id = f"{self.debug_log_prefix}-{event_id}"
        self.debug_event_counters[event_id] += 1
    
        count = self.debug_event_counters[event_id]
        self.debug_timestamps.append((timestamp, event_id, count))

        if count % 100 == 0:
            self.write_debug_timestamps(f"{self.debug_log_prefix}_timestamps.csv")

    def write_debug_timestamps(self, filename):
        with open(filename, "a") as f:
            for timestamp, event_id, count in self.debug_timestamps:
                f.write(f"{timestamp}, {event_id}, {count}\n")
        print(f"Saved debug timestamps to file: {filename}")
        self.debug_timestamps = []

    def retarget(self, joints, debug_dict=None):
        normalized_joint_pos, mano_center_and_rot = (retarget_utils.normalize_points_to_hands_local(joints))
        if self.use_franka_gripper:
            self.target_angles = self.retarget_franka_gripper_joints(normalized_joint_pos)
            debug_dict["normalized_joint_pos"] = normalized_joint_pos
        else:
            # (model_joint_pos - model_center) @ model_rotation = normalized_joint_pos
            if debug_dict is not None:
                debug_dict["mano_center_and_rot"] = mano_center_and_rot
                debug_dict["model_center_and_rot"] = (self.model_center, self.model_rotation)
                debug_dict["raw_joint_pos"] = normalized_joint_pos
            normalized_joint_pos = self.adjust_mano_fingers(normalized_joint_pos)
            normalized_joint_pos = (
                normalized_joint_pos @ self.model_rotation.T + self.model_center
            )
            if debug_dict is not None:
                debug_dict["normalized_joint_pos"] = normalized_joint_pos

            self.target_angles = self.retarget_finger_mano_joints(normalized_joint_pos)

        return self.target_angles, debug_dict

    def clean(self):
        self.gc_joints = torch.ones(self.n_tendons, device=self.device) * 15.0
        self.gc_joints.requires_grad_()
        self.opt = torch.optim.RMSprop([self.gc_joints], lr=self.lr)

    ########################
    #### Loss functions ####
    ########################

    def build_loss_function(self, loss_func_cfg):
        """
        loss_func_cfg: a dictionary of the loss function configuration. {loss_f_name: loss_f_weight}
        Each loss function should be defined as a method in the class, with the prefix "loss_"
        """
        def loss(joints: torch.Tensor, mano_points: torch.Tensor) -> torch.Tensor:
            l = 0.0
            for name, weight in loss_func_cfg.items():
                try:
                    loss_f = getattr(self, f"loss_{name}")
                except AttributeError:
                    raise ValueError(f"Loss function `loss_{name}` not found in {__class__} but required by the configuration {name}")
                if isinstance(weight, dict):
                    l += weight["weight"] * loss_f(joints, mano_points, **weight.get("kwargs", {}))
                else:
                    l += weight * loss_f(joints, mano_points)
            return l
    
        # Trace the forward function with an example input.
        robot_joints = torch.randn(self.n_tendons, device = self.device) 
        mano_points = torch.randn(22, 3, device = self.device)
        traced_loss = torch.jit.trace(loss, (robot_joints, mano_points))
        # traced_loss = torch.compile(loss, mode="reduce-overhead")
        return traced_loss
    

    def loss_keyvector_fingers2palm(self, robot_joints: torch.Tensor, mano_points: torch.Tensor) -> torch.Tensor:
        """
        Loss function for the keyvectors from the fingertips to the palm
        """
        mano_joints_dict = retarget_utils.get_mano_joints_dict(mano_points)

        mano_fingertips = {}
        mano_pps = {}
        for finger, finger_joints in mano_joints_dict.items():
            if finger == "wrist" or finger == "forearm":
                continue
            mano_fingertips[finger] = finger_joints[[-1], :]
            mano_pps[finger] = finger_joints[[0], :]

        mano_palm = torch.mean(
            torch.cat([mano_pps["thumb"], mano_pps["pinky"]], dim=0).to(self.device),
            dim=0,
            keepdim=True,
        )

        def get_keyvectors(fingertips: dict, palm: torch.Tensor):
            return torch.stack([
                fingertips["thumb"], # - palm, #palm2thumb
                fingertips["index"], # - palm, #palm2index
                fingertips["middle"], # - palm, #palm2middle
                fingertips["ring"], # - palm, #palm2ring
                fingertips["pinky"], # - palm, #palm2pinky
            ], dim=0)
        
        keyvectors_mano = get_keyvectors(mano_fingertips, mano_palm)

        chain_transforms = self.chain.forward_kinematics(
            self.joint_map @ (robot_joints / (180 / np.pi)),
            frame_indices=self.frames_we_care_about
        )
        fingertips = {}
        for finger, finger_tip in self.finger_to_tip.items():
            fingertips[finger] = transform_from_anchor(chain_transforms, finger_tip, self.anchor_name).transform_points(
                self.root
            )

        palm = (
            transform_from_anchor(chain_transforms, self.finger_to_base["thumb"], self.anchor_name).transform_points(
                self.root
            )
            + transform_from_anchor(chain_transforms, self.finger_to_base["pinky"], self.anchor_name).transform_points(
                self.root
            )
        ) / 2

        keyvectors_faive = get_keyvectors(fingertips, palm)

        return torch.sum((keyvectors_mano - keyvectors_faive) ** 2)


    def loss_keyvector_fingers2fingers(self, robot_joints: torch.Tensor, mano_points: torch.Tensor) -> torch.Tensor:
        """
        Loss function for the keyvectors from the fingertips to the palm
        """
        mano_joints_dict = retarget_utils.get_mano_joints_dict(mano_points)

        mano_fingertips = {}
        for finger, finger_joints in mano_joints_dict.items():
            if finger == "wrist" or finger == "forearm":
                continue
            mano_fingertips[finger] = finger_joints[[-1], :]


        def get_keyvectors(fingertips: dict):
            return torch.stack([
                fingertips['index'] - fingertips['thumb'], # thumb2index
                fingertips['middle'] - fingertips['thumb'], # thumb2middle
                fingertips['ring'] - fingertips['thumb'], # thumb2ring
                fingertips['pinky'] - fingertips['thumb'], # thumb2pinky
                fingertips['middle'] - fingertips['index'], # index2middle
                fingertips['ring'] - fingertips['index'], # index2ring
                fingertips['pinky'] - fingertips['index'], # index2pinky
                fingertips['ring'] - fingertips['middle'], # middle2ring
                fingertips['pinky'] - fingertips['middle'], # middle2pinky
                fingertips['pinky'] - fingertips['ring'], # ring2pinky
            ], dim=0)
        
        keyvectors_mano = get_keyvectors(mano_fingertips)

        chain_transforms = self.chain.forward_kinematics(
            self.joint_map @ (robot_joints / (180 / np.pi)),
            frame_indices=self.frames_we_care_about
        )
        fingertips = {}
        for finger, finger_tip in self.finger_to_tip.items():
            fingertips[finger] = transform_from_anchor(chain_transforms, finger_tip, self.anchor_name).transform_points(
                self.root
            )
        keyvectors_faive = get_keyvectors(fingertips)

        return torch.sum((keyvectors_mano - keyvectors_faive) ** 2)
    

    def loss_keyvector_forearm_and_wrist(self, robot_joints: torch.Tensor, mano_points: torch.Tensor) -> torch.Tensor:
        """
        Loss function for the keyvectors from the fingertips to the palm
        """
        mano_joints_dict = retarget_utils.get_mano_joints_dict(mano_points)

        forearm_and_wrist = {
            "forearm": mano_joints_dict["forearm"],
            "wrist": mano_joints_dict["wrist"]
        }

        print("forearm_and_wrist", forearm_and_wrist)
        def get_keyvectors(key_points: dict):
            print(key_points, "key_points")
            diff  = key_points['forearm'] - key_points['wrist']
            print(diff)
            return key_points['forearm'] - key_points['wrist']
            
        
        keyvectors_mano = get_keyvectors(forearm_and_wrist)

        chain_transforms = self.chain.forward_kinematics(
            self.joint_map @ (robot_joints / (180 / np.pi)),
            frame_indices=self.frames_we_care_about
        )
        key_points = {}
        for name, frame_name in self.forearm_and_wrist.items():
            key_points[name] = transform_from_anchor(chain_transforms, frame_name, self.anchor_name).transform_points(
                self.root
            )
        keyvectors_faive = get_keyvectors(key_points)

        return torch.sum((keyvectors_mano - keyvectors_faive) ** 2)


    def loss_zero_regularizor(self, robot_joints: torch.Tensor, mano_points: torch.Tensor, **joint_regularizers) -> torch.Tensor:
        """
        Regularize the joints to zero
        """
        self.regularizer_zeros = torch.zeros(self.n_tendons).to(self.device)
        self.regularizer_weights = torch.zeros(self.n_tendons).to(self.device)
        for joint_name, (zero_value, weight) in joint_regularizers.items():
            self.regularizer_zeros[self.tendon_names.index(joint_name)] = zero_value
            self.regularizer_weights[self.tendon_names.index(joint_name)] = weight

        robot_joints = robot_joints / (180 / np.pi)
        return torch.sum(
                self.regularizer_weights * (robot_joints - self.regularizer_zeros) ** 2
            )
    def loss_pinch_grasp(self, robot_joints: torch.Tensor, mano_points: torch.Tensor) -> torch.Tensor:
        """
        Loss function for the pinch grasp
        """
        mano_joints_dict = retarget_utils.get_mano_joints_dict(mano_points)

        mano_thumb2indextip = mano_joints_dict['index'][[-1],:] - mano_joints_dict['thumb'][[-1],:]
        d_mano_thumb2indextip = torch.norm(mano_thumb2indextip) # distance

        chain_transforms = self.chain.forward_kinematics(
            self.joint_map @ (robot_joints / (180 / np.pi)),
            frame_indices=self.frames_we_care_about
        )
        thumb2indextip = transform_from_anchor(chain_transforms, self.finger_to_tip['index'], self.anchor_name).transform_points(self.root) \
            - transform_from_anchor(chain_transforms, self.finger_to_tip['thumb'], self.anchor_name).transform_points(self.root)
        d_thumb2indextip = torch.norm(thumb2indextip) # distance
        # loss = torch.abs((d_mano_thumb2indextip - d_thumb2indextip)) / (d_mano_thumb2indextip + 1e-2) # normalized loss
        loss = torch.sum((d_mano_thumb2indextip - d_thumb2indextip)**2) / (d_mano_thumb2indextip + 1e-2) # normalized loss
        return loss

    def loss_virtual_coupling(self, robot_joints: torch.Tensor, mano_points: torch.Tensor, **couplings) -> torch.Tensor:
        """
        Loss function for the virtual coupling.
        couplings: a dictionary of the coupling weights and the joint names, e.g
        """
        loss = torch.tensor(0.0).to(self.device)
        robot_joints = robot_joints / (180 / np.pi)

        for cname, (weight, joint_lists) in couplings.items():
            factors = torch.zeros(self.n_tendons).to(self.device)
            for factor, joint_name in joint_lists: 
                factors[self.tendon_names.index(joint_name)] = factor
            loss += (factors * robot_joints).sum()**2 * weight
        return loss