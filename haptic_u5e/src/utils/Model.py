#!/usr/bin/env python3
import csv
import cv2 as cv

class Model:
    mode = 0
    number = 0

    @staticmethod
    def select_mode(key):
        Model.number = -1
        if 48 <= key <= 57:  # 0 ~ 9
            Model.number = key - 48
        if 97 <= key <= 103:  # a ~ g
            Model.number = key - 87
            print("Number: ", Model.number)
        if key == 110:  # n
            Model.mode = 0
        if key == 107:  # k
            Model.mode = 1
        if key == 104:  # h
            Model.mode = 2

    @staticmethod
    def logging_csv(landmark_list, point_history_list):
        if Model.mode == 0:
            pass
        if Model.mode == 1 and (0 <= Model.number <= 16):
            csv_path = '/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/keypoint_classifier/keypoint.csv'
            print("Logging: Num ", Model.number, " Mode ", Model.mode)
            with open(csv_path, 'a', newline="") as f:
                writer = csv.writer(f)
                writer.writerow([Model.number, *landmark_list])
        if Model.mode == 2 and (0 <= Model.number <= 9):
            csv_path = '/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/point_history_classifier/point_history.csv'
            with open(csv_path, 'a', newline="") as f:
                writer = csv.writer(f)
                writer.writerow([Model.number, *point_history_list])
        return
    
    @staticmethod
    def draw_mode_info(image):
        mode_string = ['Logging Key Point', 'Logging Point History']
        if 1 <= Model.mode <= 2:
            cv.putText(image, "MODE:" + mode_string[Model.mode - 1], (10, 90),
                    cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
            if 0 <= Model.number <= 9:
                cv.putText(image, "NUM:" + str(Model.number), (10, 110),
                        cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
        

    