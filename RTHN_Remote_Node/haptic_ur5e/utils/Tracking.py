#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import mediapipe as mp
from utils.Classifier import Classifier

class Tracking:

    def __init__(self, args):
        
        #self.point_reference = [0.5, 0.5, 0.01] #[0.5, 0.15, 0.01] #[0.5, 0.5, 0.01]
        #self.tolerance = [0.15, 0.15, 0.06] #[0.5, 0.5, 0.06] #[0.15, 0.15, 0.06]
        self.brect = None

        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=args.use_static_image_mode,
            max_num_hands=1,
            min_detection_confidence=args.min_detection_confidence,
            min_tracking_confidence=args.min_tracking_confidence,
        )

        self.classifier = Classifier()
        self.configure_point_reference(args.initial_position)
        self.configure_tolerance(args.tolerance)
        self.hand_sign_id = None
        self.mean_landmarks_coords = [0.0, 0.0, 0.0]
        self.mean_landmarks_list = [0, 0]
        self.results = None

    def configure_point_reference(self, initial_pos):
        self.point_reference = [0.5, 0.5, 0.01] # CASE 04: mid_centered
        if initial_pos == 0:
            self.point_reference = [0.2, 0.2, 0.01] # CASE 00: up_left
        elif initial_pos == 1:
            self.point_reference = [0.5, 0.2, 0.01] # CASE 01: up_centered
        elif initial_pos == 2:
            self.point_reference = [0.8, 0.2, 0.01] # CASE 02: up_right
        elif initial_pos == 3:
            self.point_reference = [0.2, 0.5, 0.01] # CASE 03: mid_left
        elif initial_pos == 5:
            self.point_reference = [0.8, 0.5, 0.01] # CASE 05: mid_right
        elif initial_pos == 6:
            self.point_reference = [0.2, 0.8, 0.01] # CASE 06: lower_left
        elif initial_pos == 7:
            self.point_reference = [0.5, 0.8, 0.01] # CASE 07: lower_mid
        elif initial_pos == 8:
            self.point_reference = [0.8, 0.8, 0.01] # CASE 08: lower_right

    def configure_tolerance(self, tol_pos):
        self.tolerance = [0.15, 0.15, 0.06] # CASE 01: MEDIUM
        if tol_pos == 0:
            self.tolerance = [0.08, 0.08, 0.03] # CASE 00: LOWER
        elif tol_pos == 2:
            self.tolerance = [0.22, 0.22, 0.09] # CASE 02: HIGHER

    def configure_detection(self, image, gloves_color):
        if gloves_color == 1: # BLUE GLOVES
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        elif gloves_color == 2: # YELLOW GLOVES
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB) # MODIFY with HSV
        elif gloves_color == 3: # BLACK GLOVES
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB) # MODIFY with HSV
        return image

    def process_tracking(self, image):
        self.results = self.hands.process(image)

    def is_grip_happening(self):
        return self.hand_sign_id in range(6,25,1) or self.hand_sign_id == 1
    
    def detect_grip_level(self):
        grip_level = 0
        if self.hand_sign_id in range(17,21,1): # SOFT Grip
            grip_level = 1
        elif self.hand_sign_id in range(13,17,1): # Normal Grip
            grip_level = 2
        elif self.hand_sign_id in range(21,25,1): # HARD Grip
            grip_level = 3
        elif self.hand_sign_id == 1: # CLOSE Grip
            grip_level = 4
        return grip_level
    
    def detect_grip_fingers(self):
        grip_fingers = 0
        if self.hand_sign_id in range(12,25,4): # 2 Fingers
            grip_fingers = 2
        elif self.hand_sign_id in range(11,25,4): # 3 Fingers
            grip_fingers = 3
        elif self.hand_sign_id in range(10,25,4): # 4 Fingers
            grip_fingers = 4
        elif self.hand_sign_id in range(9,25,4): # 5 Fingers
            grip_fingers = 5
        elif self.hand_sign_id == 1: # CLOSE Grip
            grip_fingers = 6
        return grip_fingers

    def configure_hand_sign_id(self, pre_processed_landmark_list):
        self.hand_sign_id = self.classifier.configure_keypoints(pre_processed_landmark_list)

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
        return self.classifier.get_keypoint_labels(self.hand_sign_id)
    
    def set_new_mean_landmarks_coords(self, all_coords, len_coord):
        self.mean_landmarks_coords[0] = round(sum(all_coords[0])/len_coord,3)
        self.mean_landmarks_coords[1] = round(sum(all_coords[1])/len_coord,3)
        self.mean_landmarks_coords[2] = round(sum(all_coords[2])/len_coord,3)

    def get_keypoint_classifier(self, landmark_list):
        return self.classifier.keypoint_classifier(landmark_list)
    
    def get_point_history_classifier(self, pre_processed_point_history_list):
        return self.classifier.point_history_classifier(pre_processed_point_history_list)