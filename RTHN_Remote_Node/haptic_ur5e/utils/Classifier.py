#!/usr/bin/env python3

import csv
from model import KeyPointClassifier, PointHistoryClassifier

class Classifier:

    def __init__(self):
        
        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier()

        with open('C:/Users/rduser/Documents/FernandoHernandez/haptic_ur5e/model/keypoint_classifier/keypoint_classifier_label.csv',
                encoding='utf-8-sig') as f:
            self.keypoint_classifier_labels = csv.reader(f)
            self.keypoint_classifier_labels = [
                row[0] for row in self.keypoint_classifier_labels
            ]
        with open('C:/Users/rduser/Documents/FernandoHernandez/haptic_ur5e/model/point_history_classifier/point_history_classifier_label.csv',
                encoding='utf-8-sig') as f:
            self.point_history_classifier_labels = csv.reader(f)
            self.point_history_classifier_labels = [
                row[0] for row in self.point_history_classifier_labels
            ]

    def get_keypoint_labels(self, hand_sign_id):
        return self.keypoint_classifier_labels[hand_sign_id]
    
    def configure_keypoints(self, pre_processed_landmark_list):
        return self.keypoint_classifier(pre_processed_landmark_list)