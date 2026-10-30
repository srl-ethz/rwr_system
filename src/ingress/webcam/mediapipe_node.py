#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from webcam_mediapipe_ingress import MediaPipeTracker
from faive_system.src.common.utils import numpy_to_float32_multiarray
import cv2


class MediapipeNode(Node):
    def __init__(self):
        super().__init__("mediapipe_node")

        self.tracker = MediaPipeTracker(render_annotated_frame=True)
        self.tracker.start()

        self.bridge = CvBridge()

        ingress_hz = 40.
        self.timer = self.create_timer(1.0 / ingress_hz, self.timer_publish_cb)

        annotated_frame_hz = 10.
        self.timer_annotated_frame = self.create_timer(
            1.0 / annotated_frame_hz, self.timer_publish_annotated_frame_cb
        )

        self.ingress_mano_pub = self.create_publisher(
            Float32MultiArray, "/ingress/right/mano", 10
        )
        self.annotated_image_pub = self.create_publisher(
            Image, "/ingress/right/mediapipe/annotated_image", 10
        )

        # Scale factor for the annotated image publish to improve throughput
        self.annotated_frame_scale = 0.5
    
    def normalize_scale(self, keypoints: np.ndarray) -> np.ndarray:
        """
        Sometimes the scale of the detected hand jitters (especially when some fingers are occluded)
        """
        assert keypoints.shape[0] == 21
        wrist = keypoints[0]
        index_base = keypoints[5]
        middle_base = keypoints[9]
        ring_base = keypoints[13]
        pinky_base = keypoints[17]
        average_metacarpal_length = np.mean([
            np.linalg.norm(wrist - index_base),
            np.linalg.norm(wrist - middle_base),
            np.linalg.norm(wrist - ring_base),
            np.linalg.norm(wrist - pinky_base),
        ])
        target_metacarpal_length = 0.09  # somewhat arbitrary magic number
        keypoints = (keypoints - wrist) * (target_metacarpal_length / average_metacarpal_length) + wrist

        return keypoints

    
    def add_forearm_keypoint(self, keypoints: np.ndarray) -> np.ndarray:
        """Append a forearm keypoint to the MANO layout.

        Mediapipe provides 21 hand landmarks where index 0 is the wrist and
        9 corresponds to the middle finger base. The retargeter expects an
        additional forearm landmark in front of the wrist (index 0 in the
        MANO layout). We synthesize this point by extending the vector from
        the middle finger base to the wrist beyond the wrist by the same
        magnitude.

        Args:
            keypoints: Array of shape (21, 3) from Mediapipe.

        Returns:
            Array of shape (22, 3) with the synthetic forearm point prepended.
        """

        if keypoints.shape[0] != 21:
            raise ValueError("Expected 21 Mediapipe keypoints")

        wrist = keypoints[0]
        middle_base = keypoints[9]
        forearm = wrist + (wrist - middle_base)
        return np.vstack((forearm, keypoints))


    def timer_publish_cb(self):
        keypoint_positions_wo_wrist = self.tracker.get_keypoint_positions()
        if keypoint_positions_wo_wrist is not None:
            keypoint_positions_wo_wrist = self.normalize_scale(keypoint_positions_wo_wrist)
            keypoint_positions = self.add_forearm_keypoint(keypoint_positions_wo_wrist)
            keypoint_positions_msg = numpy_to_float32_multiarray(keypoint_positions)
            self.ingress_mano_pub.publish(keypoint_positions_msg)
    
    def timer_publish_annotated_frame_cb(self):

        annotated_frame = self.tracker.get_annotated_frame()
        if annotated_frame is not None:
            try:
                # Resize before publishing to improve throughput
                if self.annotated_frame_scale != 1.0:
                    assert 0. < self.annotated_frame_scale < 1.0
                    annotated_frame = cv2.resize(
                        annotated_frame, None, fx=self.annotated_frame_scale, fy=self.annotated_frame_scale, interpolation=cv2.INTER_AREA
                    )

                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "webcam"
                image_msg = self.bridge.cv2_to_imgmsg(
                    annotated_frame, "bgr8", header=header
                )
                self.annotated_image_pub.publish(image_msg)
            except CvBridgeError as exc:
                self.get_logger().error(f"Error publishing annotated frame: {exc}")

    def destroy_node(self):
        self.tracker.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MediapipeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
