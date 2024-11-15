import numpy as np
from typing import Dict

# the information of the tendons in the hand. Each tendon represents a grouped actuation.
GC_TENDONS = {
    'root2thumb_base': {},
    'thumb_base2pp': {},
    'thumb_pp2mp': {},
    'thumb_mp2dp': {},
    'index_abd': {},
    'root2index_pp': {},
    'index_pp2mp': {'index_mp2dp': 0.58333},
    'middle_abd': {},
    'root2middle_pp': {},
    'middle_pp2mp': {'middle_mp2dp': 0.58333},
    'ring_abd': {},
    'root2ring_pp': {},
    'ring_pp2mp': {'ring_mp2dp': 0.58333},
    'pinky_abd': {},
    'root2pinky_pp': {},
    'pinky_pp2mp': {'pinky_mp2dp': 0.58333},
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
    "index": "index_um",
    "middle": "middle_um",
    "ring": "ring_um",
    "pinky": "pinky_um",
}

GC_LIMITS_LOWER = np.array(
    [
        -57.0,  # root2thumb_base
        0.0,  # thumb_base2pp
        0.0,  # thumb_pp2mp
        0.0,  # thumb_mp2dp
        0.0,  # index_abd
        0.0,  # root2index_pp
        0.0,  # index_pp2mp
        0.0,  # middle_abd
        0.0,  # root2middle_pp
        0.0,  # middle_pp2mp
        0.0,  # ring_abd
        0.0,  # root2ring_pp
        0.0,  # ring_pp2mp
        0.0,  # pinky_abd
        0.0,  # root2pinky_pp
        0.0,  # pinky_pp2mp
    ]
)
GC_LIMITS_UPPER = np.array(
    [
        40.1,  # root2thumb_base
        179.9,  # thumb_base2pp
        85.9,  # thumb_pp2mp
        91.7,  # thumb_mp2dp
        54.4,  # index_abd
        108.8,  # root2index_pp
        94.5,  # index_pp2mp
        54.4,  # middle_abd
        108.8,  # root2middle_pp
        94.5,  # middle_pp2mp
        54.4,  # ring_abd
        108.8,  # root2ring_pp
        94.5,  # ring_pp2mp
        54.4,  # pinky_abd
        108.8,  # root2pinky_pp
        94.5,  # pinky_pp2mp
    ]
)
