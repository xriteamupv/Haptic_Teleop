#!/usr/bin/env python3
import copy
import itertools
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
    
    def pre_process_landmark(self, landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)
        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]
            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y
        temp_landmark_list = list(
            itertools.chain.from_iterable(temp_landmark_list))
        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))
        return temp_landmark_list
    
    def pre_process_point_history(self, img_dimensions):
        temp_point_history = copy.deepcopy(self.point_history)

        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, point in enumerate(temp_point_history):
            if index == 0:
                base_x, base_y = point[0], point[1]

            temp_point_history[index][0] = (temp_point_history[index][0] -
                                            base_x) / img_dimensions[0]
            temp_point_history[index][1] = (temp_point_history[index][1] -
                                            base_y) / img_dimensions[1]

        # Convert to a one-dimensional list
        temp_point_history = list(itertools.chain.from_iterable(temp_point_history))
        return temp_point_history