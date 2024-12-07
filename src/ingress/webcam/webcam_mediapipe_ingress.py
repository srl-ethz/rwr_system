import cv2
import mediapipe as mp
import threading
import time
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose


class MediaPipeTracker:
    def __init__(self, show=False):
        self.cap = cv2.VideoCapture(0)
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.pose = mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.keypoint_positions = None
        self.estimated_wrist_position = None  # Extra wrist point
        self.elbow_position = None  # Elbow position
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
        if self.estimated_wrist_position is not None and self.elbow_position is not None:
            z = self.keypoint_positions[0][2]
            self.estimated_wrist_position = np.array([self.estimated_wrist_position[0],self.estimated_wrist_position[1],z])
            return np.vstack((self.estimated_wrist_position, self.keypoint_positions))
        else:
            return None

    def run_tracker(self):
        while self.running and self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Process hand and pose landmarks
            hand_results = self.hands.process(image)
            pose_results = self.pose.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            self.elbow_position = None  # Reset elbow position
            self.estimated_wrist_position = None  # Reset wrist position

            if pose_results.pose_landmarks:
                # Extract elbow position from pose
                landmarks = pose_results.pose_landmarks.landmark
                right_elbow = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value]
                self.elbow_position = np.array([right_elbow.x, right_elbow.y, right_elbow.z])

            
            if hand_results.multi_hand_landmarks:
                for hand_landmarks, hand_handedness in zip(
                    hand_results.multi_hand_landmarks, hand_results.multi_handedness
                ):
                    if hand_handedness.classification[0].label == "Left":  # Use "Right" hand labels are reversed
                        # Extract keypoint positions
                        self.keypoint_positions = np.array(
                            [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark],
                            dtype=np.float32,
                        )

                        # Calculate the wrist and extrapolate
                        wrist = hand_landmarks.landmark[0]

                        if self.elbow_position is not None:
                            elbow_to_wrist = np.array([wrist.x, wrist.y, wrist.z]) - self.elbow_position
                            self.estimated_wrist_position = np.array([wrist.x, wrist.y, wrist.z]) - 0.4 * elbow_to_wrist

                        # Draw landmarks and the estimated wrist point
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style(),
                        )

                        # Draw the estimated wrist position
                        if self.estimated_wrist_position is not None:
                            cv2.circle(
                                image,
                                (int(self.estimated_wrist_position[0] * image.shape[1]),
                                 int(self.estimated_wrist_position[1] * image.shape[0])),
                                5,
                                (255, 0, 0),  # Blue circle
                                -1,
                            )

                        # Draw the elbow position
                        if self.elbow_position is not None:
                            cv2.circle(
                                image,
                                (int(self.elbow_position[0] * image.shape[1]),
                                 int(self.elbow_position[1] * image.shape[0])),
                                5,
                                (0, 255, 0),  # Green circle
                                -1,
                            )

            # Update the frame for display
            self.frame = cv2.flip(image, 1) if self.show else None
        self.cap.release()

def main():
    tracker = MediaPipeTracker(show=True)
    tracker.start()
    try:
        while tracker.running:
            if tracker.frame is not None:
                cv2.imshow("MediaPipe Tracker", tracker.frame)
                if cv2.waitKey(5) & 0xFF == 27:
                    tracker.stop()
                    break

            keypoint_positions = tracker.get_keypoint_positions()
            if keypoint_positions is not None:
                print("Keypoints and Wrist Position:", keypoint_positions)
            else:
                if tracker.elbow_position is None:
                    print("Elbow position not found.")
                elif tracker.estimated_wrist_position is None:
                    print("Wrist position not found.")
            time.sleep(0.01)  # Small delay to avoid busy-waiting
    finally:
        tracker.stop()


if __name__ == "__main__":
    main()
