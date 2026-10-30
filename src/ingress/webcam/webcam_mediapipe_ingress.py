import cv2
import mediapipe as mp
import threading
import time
from typing import Optional

import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

class MediaPipeTracker:
    def __init__(self, render_annotated_frame: bool = False, camera_index: int = 0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.keypoint_positions = None
        self.running = False
        self.frame: Optional[np.ndarray] = None  # Shared frame for display or publishing
        self._frame_lock = threading.Lock()
        self.render_annotated_frame = render_annotated_frame
        self.thread = threading.Thread(target=self.run_tracker)
    
    def start(self):
        self.running = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
    
    def get_keypoint_positions(self):
        return self.keypoint_positions

    def get_annotated_frame(self) -> Optional[np.ndarray]:
        """Return a copy of the latest annotated frame if available."""
        if not self.render_annotated_frame:
            return None
        with self._frame_lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def run_tracker(self):
        while self.running and self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.hands.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.multi_hand_world_landmarks:
                for hand_landmarks, hand_handedness in zip(results.multi_hand_world_landmarks, results.multi_handedness):
                    # The labels are reversed since it assumes the input is mirrored image (like a selfie camera)
                    # https://mediapipe.readthedocs.io/en/latest/solutions/hands.html#multi-handedness
                    if hand_handedness.classification[0].label == 'Left':
                        self.keypoint_positions = np.array([(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark], dtype=np.float32)
                    else:
                        self.keypoint_positions = None
            # Update the frame for display or publishing
            if results.multi_hand_landmarks and self.render_annotated_frame:
                for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style()
                        )
            annotated_image = cv2.flip(image, 1)
            with self._frame_lock:
                self.frame = annotated_image
        self.cap.release()

def main():
    tracker = MediaPipeTracker(render_annotated_frame=True)
    tracker.start()
    try:
        while tracker.running:
            frame = tracker.get_annotated_frame()
            if frame is not None:
                cv2.imshow('MediaPipe Hands', frame)
                if cv2.waitKey(5) & 0xFF == 27:
                    tracker.stop()
                    break
            keypoint_positions = tracker.get_keypoint_positions()
            if keypoint_positions is not None:
                print(keypoint_positions)
            time.sleep(0.01)  # Small delay to avoid busy-waiting
    finally:
        tracker.stop()

if __name__ == "__main__":
    main()
