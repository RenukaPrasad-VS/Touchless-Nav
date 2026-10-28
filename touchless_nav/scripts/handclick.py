import cv2
import mediapipe as mp
import time

class HandClickDetector:
    """Detect an air click when an extended index finger pokes forward."""
    def __init__(self, max_num_hands=1, detection_conf=0.7, tracking_conf=0.7):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=max_num_hands,
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        self.click_state = False
        self.last_click_time = 0
        self.depth_baseline = None
        self.cooldown = 0.4  # seconds between clicks

    def process_frame(self, frame):
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        click_coords = None

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                landmarks = hand_landmarks.landmark
                index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
                wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
                middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

                palm_size = ((wrist.x - middle_mcp.x) ** 2
                             + (wrist.y - middle_mcp.y) ** 2) ** 0.5
                finger_length = ((index_tip.x - index_mcp.x) ** 2
                                 + (index_tip.y - index_mcp.y) ** 2) ** 0.5
                index_is_extended = finger_length / max(palm_size, 1e-6) > 0.45

                # Track fingertip depth relative to its MCP joint, so moving
                # the whole hand toward the camera doesn't count as a click.
                relative_depth = index_tip.z - index_mcp.z
                if self.depth_baseline is None:
                    self.depth_baseline = relative_depth
                depth_delta = relative_depth - self.depth_baseline

                was_clicking = self.click_state
                if self.click_state:
                    # Pull the fingertip back to re-arm the next click.
                    self.click_state = depth_delta < -0.015
                else:
                    self.click_state = index_is_extended and depth_delta < -0.035

                if (self.click_state and not was_clicking
                        and time.time() - self.last_click_time > self.cooldown):
                    self.last_click_time = time.time()
                    click_coords = (int(index_tip.x * w), int(index_tip.y * h))
                elif not self.click_state:
                    # Slowly adapt to changes in hand angle while waiting for
                    # the deliberate forward finger motion.
                    self.depth_baseline = (0.9 * self.depth_baseline
                                           + 0.1 * relative_depth)

                color = (0, 255, 0) if self.click_state else (0, 0, 255)
                x, y = int(index_tip.x * w), int(index_tip.y * h)
                cv2.circle(frame, (x, y), 8, color, cv2.FILLED)
        else:
            self.click_state = False
            self.depth_baseline = None

        return frame, click_coords
