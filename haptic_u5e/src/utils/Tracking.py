#!/usr/bin/env python3

import csv
import copy
import itertools

import cv2 as cv
import numpy as np
import mediapipe as mp
from model import KeyPointClassifier, PointHistoryClassifier

class Tracking:

    def __init__(self, args):
        
        self.point_reference = [0.5, 0.5, 0.01] #[0.5, 0.15, 0.01] #[0.5, 0.5, 0.01]
        self.tolerance = [0.15, 0.15, 0.06] #[0.5, 0.5, 0.06] #[0.15, 0.15, 0.06]
        self.brect = None

        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=args.use_static_image_mode,
            max_num_hands=1,
            min_detection_confidence=args.min_detection_confidence,
            min_tracking_confidence=args.min_tracking_confidence,
        )

        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier()
        self.hand_sign_id = None

        with open('/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/keypoint_classifier/keypoint_classifier_label.csv',
                encoding='utf-8-sig') as f:
            self.keypoint_classifier_labels = csv.reader(f)
            self.keypoint_classifier_labels = [
                row[0] for row in self.keypoint_classifier_labels
            ]
        with open('/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/point_history_classifier/point_history_classifier_label.csv',
                encoding='utf-8-sig') as f:
            self.point_history_classifier_labels = csv.reader(f)
            self.point_history_classifier_labels = [
                row[0] for row in self.point_history_classifier_labels
            ]
        self.mean_landmarks_coords = [0.0, 0.0, 0.0]
        self.mean_landmarks_list = [0, 0]
        self.results = None

    def process_tracking(self, image):
        self.results = self.hands.process(image)

    def configure_hand_sign_id(self, pre_processed_landmark_list):
        self.hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)

    def is_hand_near_reference(self):
        is_near_reference = [abs(self.mean_landmarks_coords[i] - self.point_reference[i]) <= self.tolerance[i] for i in range(3)]
        return all(is_near_reference)
    
    def get_bounding_rect(self, landmarks, img_dimensions):
        landmark_array = np.empty((0, 2), int)
        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * img_dimensions[0]), img_dimensions[0] - 1)
            landmark_y = min(int(landmark.y * img_dimensions[1]), img_dimensions[1] - 1)
            landmark_point = [np.array((landmark_x, landmark_y))]
            landmark_array = np.append(landmark_array, landmark_point, axis=0)
        x, y, w, h = cv.boundingRect(landmark_array)
        self.brect = [x, y, x + w, y + h]
    
    def get_landmark_list(self, landmarks, img_dimensions):
        landmark_coord = []
        landmark_point = []
        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * img_dimensions[0]), img_dimensions[0] - 1)
            landmark_y = min(int(landmark.y * img_dimensions[1]), img_dimensions[1] - 1)
            landmark_z = landmark.z
            landmark_point.append([landmark_x, landmark_y])
            landmark_coord.append([landmark.x, landmark.y, landmark.z])
        return landmark_point, landmark_coord
    
    def get_keypoint_labels(self):
        return self.keypoint_classifier_labels[self.hand_sign_id]
    
    def set_new_mean_landmarks_coords(self, all_coords, len_coord):
        self.mean_landmarks_coords[0] = round(sum(all_coords[0])/len_coord,3)
        self.mean_landmarks_coords[1] = round(sum(all_coords[1])/len_coord,3)
        self.mean_landmarks_coords[2] = round(sum(all_coords[2])/len_coord,3)
    
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
    
    def pre_process_point_history(self, point_history, img_dimensions):
        temp_point_history = copy.deepcopy(point_history)

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