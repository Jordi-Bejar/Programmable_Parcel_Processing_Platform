import cv2
import numpy as np

def capture_frame(cap):
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to capture frame.")
        return None
    return frame

def detect_box(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_color = np.array([0, 100, 100])
    upper_color = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_color, upper_color)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("[WARN] No box detected.")
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)

    if M["m00"] == 0:
        return None

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return (cX, cY)

class BoxDetector:
    def __init__(self, color_lower, color_upper):
        self.color_lower = np.array(color_lower)
        self.color_upper = np.array(color_upper)

    def capture_frame(self, cap):
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to capture frame.")
            return None
        return frame

    def detect_box(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            print("[WARN] No box detected.")
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] == 0:
            return None

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        return (cX, cY)