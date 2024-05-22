#!/usr/bin/env python3
from collections import Counter
from collections import deque

class History:

    def __init__(self):
        self.history_length = 16
        self.point_history = deque(maxlen=self.history_length)
        self.finger_gesture_history = deque(maxlen=self.history_length)
        self.pre_processed_landmark_list = None
        self.pre_processed_point_history_list = None

    def empty_point_history(self):
        self.point_history.append([0, 0])

    def append_point_history(self, landmarks):
        self.point_history.append(landmarks)

    def append_gesture(self, gesture_id):
        self.finger_gesture_history.append(gesture_id)

    def find_most_common_gesture(self):
        return Counter(self.finger_gesture_history).most_common()