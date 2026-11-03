import cv2
from collections import deque
import mediapipe as mp
import time

class HandClickDetector:
    """Detect a deliberate forward index-finger poke using relative landmark depth."""
    def __init__(self, max_num_hands=1, detection_conf=0.7, tracking_conf=0.7,
                 poke_threshold=0.04, release_threshold=0.015,
                 min_poke_frames=2, cooldown=0.5):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=max_num_hands,
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        self.depth_history = deque(maxlen=5)
        self.baseline = None
        self.click_state = False
        self.poke_frames = 0
        self.last_click_time = 0.0
        self.poke_threshold = float(poke_threshold)
        self.release_threshold = float(release_threshold)
        self.min_poke_frames = max(1, int(min_poke_frames))
        self.cooldown = float(cooldown)

    def _reset_tracking(self):
        self.depth_history.clear()
        self.baseline = None
        self.click_state = False
        self.poke_frames = 0

    def process_frame(self, frame):
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        click_coords = None

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            self.mp_drawing.draw_landmarks(
                frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
            x, y = int(index_tip.x * w), int(index_tip.y * h)
            # MediaPipe z is relative and normalized. Subtracting wrist z
            # reduces the effect of moving the whole hand toward the camera.
            self.depth_history.append(index_tip.z - wrist.z)
            depth = sorted(self.depth_history)[len(self.depth_history) // 2]
            if self.baseline is None:
                self.baseline = depth
            now = time.monotonic()
            depth_delta = depth - self.baseline

            if not self.click_state:
                if depth_delta < -self.poke_threshold:
                    self.poke_frames += 1
                else:
                    self.poke_frames = 0
                    # Slowly follow natural pose drift only while rearmed.
                    self.baseline = 0.95 * self.baseline + 0.05 * depth
                if (self.poke_frames >= self.min_poke_frames
                        and now - self.last_click_time >= self.cooldown):
                    self.click_state = True
                    self.last_click_time = now
                    click_coords = (x, y)
            elif depth_delta > -self.release_threshold:
                self.click_state = False
                self.poke_frames = 0
                self.baseline = depth

            color = (0, 255, 0) if self.click_state else (0, 0, 255)
            cv2.circle(frame, (x, y), 8, color, cv2.FILLED)
        else:
            # Require a fresh baseline after tracking is lost; this prevents a
            # held pose from creating a click as soon as the hand reappears.
            self._reset_tracking()

        return frame, click_coords

    def close(self):
        self.hands.close()
