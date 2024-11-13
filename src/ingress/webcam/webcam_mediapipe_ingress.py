import cv2
import mediapipe as mp
import threading
import time
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

class MediaPipeTracker:
    def __init__(self, show=False):
        self.cap = cv2.VideoCapture(0)
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.keypoint_positions = None
        self.show = show
        self.running = False
        self.frame = None  # Shared frame for display in main thread
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
            if results.multi_hand_landmarks:
                for hand_landmarks, hand_handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    if hand_handedness.classification[0].label == 'Left': # For some reason, the labels are reversed, TODO: Investigate
                        self.keypoint_positions = np.array([(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark], dtype=np.float32)
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style()
                        )
                    else:
                        self.keypoint_positions = None
            # Update the frame for display
            self.frame = cv2.flip(image, 1) if self.show else None
        self.cap.release()

def main():
    tracker = MediaPipeTracker(show=True)
    tracker.start()
    try:
        while tracker.running:
            if tracker.frame is not None:
                cv2.imshow('MediaPipe Hands', tracker.frame)
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
