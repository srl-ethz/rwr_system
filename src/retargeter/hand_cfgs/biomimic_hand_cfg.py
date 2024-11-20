import numpy as np
from typing import Dict

# the information of the tendons in the hand. Each tendon represents a grouped actuation.
GC_TENDONS = {
    # "root2palm": {},
    "root2thumb_base": {},
    "thumb_base2pp": {},
    "thumb_pp2mp_virt": {
        "thumb_pp2mp": 1,
        "thumb_mp2dp_virt": 0.71,
        "thumb_mp2dp": 0.71,
    },
    "index_base2adb_virt": {"index_base2adb": 1},
    "index_adb2pp_virt": {"index_adb2pp": 1},
    "index_pp2mp_virt": {
        "index_pp2mp": 1,
        "index_mp2dp_virt": 0.71,
        "index_mp2dp": 0.71,
    },
    "middle_base2adb_virt": {"middle_base2adb": 1},
    "middle_adb2pp_virt": {"middle_adb2pp": 1},
    "middle_pp2mp_virt": {
        "middle_pp2mp": 1,
        "middle_mp2dp_virt": 0.71,
        "middle_mp2dp": 0.71,
    },
    "ring_base2adb_virt": {"ring_base2adb": 1},
    "ring_adb2pp_virt": {"ring_adb2pp": 1},
    "ring_pp2mp_virt": {
        "ring_pp2mp": 1,
        "ring_mp2dp_virt": 0.71,
        "ring_mp2dp": 0.71
    },
    "pinky_base2adb_virt": {"pinky_base2adb": 1},
    "pinky_adb2pp_virt": {"pinky_adb2pp": 1},
    "pinky_pp2mp_virt": {
        "pinky_pp2mp": 1,
        "pinky_mp2dp_virt": 0.71,
        "pinky_mp2dp": 0.71,
    },
}

# the mapping from fingername to the frame of the fingertip
# Use pytorch_kinematics.Chain.print_tree() to see the tip frame
FINGER_TO_TIP: Dict[str, str] = {
    "thumb": "thumb_fingertip",
    "index": "index_fingertip",
    "middle": "middle_fingertip",
    "ring": "ring_fingertip",
    "pinky": "pinky_fingertip",
}

# the mapping from fingername to the frame of the fingerbase (The base that fixed to the palm)
# Use pytorch_kinematics.Chain.print_tree() to see the base frame
FINGER_TO_BASE = {
    "thumb": "thumb_base",
    "index": "index_base",
    "middle": "middle_base",
    "ring": "ring_base",
    "pinky": "pinky_base",
}

GC_LIMITS_LOWER = np.array(
    [
        0.0,  # root2thumb_base
        -45.0,  # thumb_base2pp
        -90.0,  # thumb_pp2mp_virt
        -30.0,  # index_base2abd_virt
        0.0,  # index_adb2pp_virt
        0.0,  # index_pp2mp_virt
        -30.0,  # middle_base2abd_virt
        0.0,  # middle_adb2pp_virt
        0.0,  # middle_pp2mp_virt
        -30.0,  # ring_base2abd_virt
        0.0,  # ring_adb2pp_virt
        0.0,  # ring_pp2mp_virt
        -30.0,  # pinky_base2abd_virt
        0.0,  # pinky_adb2pp_virt
        0.0,  # pinky_pp2mp_virt
    ]
)
GC_LIMITS_UPPER = np.array(
    [
        70.0,  # root2thumb_base
        45.0,  # thumb_base2pp
        0.0,  # thumb_pp2mp_virt
        30.0,  # index_base2adb_virt
        90.0,  # index_adb2pp_virt
        90.0,  # index_pp2mp_virt
        30.0,  # middle_base2abd_virt
        90.0,  # middle_adb2pp_virt
        90.0,  # middle_pp2mp_virt
        30.0,  # ring_base2adb_virt
        90.0,  # ring_adb2pp_virt
        90.0,  # ring_pp2mp_virt
        30.0,  # pinky_base2adb_virt
        90.0,  # pinky_adb2pp_virt
        90.0,  # pinky_pp2mp_virt
    ]
)
