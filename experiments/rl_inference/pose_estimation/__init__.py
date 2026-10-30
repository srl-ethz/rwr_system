"""
Pose Estimation Package for Cube Detection using Mask R-CNN
"""

from .maskrcnn_detection import maskrcnn_detection, load_detector, DetectorMaskRCNN

__version__ = "0.1.0"

__all__ = [
    "maskrcnn_detection",
    "load_detector", 
    "DetectorMaskRCNN",
]
