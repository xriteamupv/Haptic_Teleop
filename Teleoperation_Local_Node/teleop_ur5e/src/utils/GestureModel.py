#!/usr/bin/env python3
import csv
import cv2 as cv

class GestureModel:
    mode = 0
    number = 0

    @staticmethod
    def select_mode(key):
        GestureModel.number = -1
        if 48 <= key <= 57:  # 0 ~ 9
            GestureModel.number = key - 48
        if 97 <= key <= 119:  # a ~ w = 10 ~ 33
            GestureModel.number = key - 87
            print("Number: ", GestureModel.number)
        if key == 120:  # x
            GestureModel.mode = 0
        if key == 121:  # y
            GestureModel.mode = 1
        if key == 122:  # z
            GestureModel.mode = 2

    @staticmethod
    def logging_csv(landmark_list, point_history_list):
        if GestureModel.mode == 0:
            pass
        if GestureModel.mode == 1 and (0 <= GestureModel.number <= 33):
            csv_path = '/home/ros-iteam/catkin_ws/src/haptic_ur5e/src/model/keypoint_classifier/keypoint.csv'
            print("Logging: Num ", GestureModel.number, " Mode ", GestureModel.mode)
            with open(csv_path, 'a', newline="") as f:
                writer = csv.writer(f)
                writer.writerow([GestureModel.number, *landmark_list])
        if GestureModel.mode == 2 and (0 <= GestureModel.number <= 9):
            csv_path = '/home/ros-iteam/catkin_ws/src/haptic_ur5e/src/model/point_history_classifier/point_history.csv'
            with open(csv_path, 'a', newline="") as f:
                writer = csv.writer(f)
                writer.writerow([GestureModel.number, *point_history_list])
        return
    
    @staticmethod
    def draw_mode_info(image):
        mode_string = ['Logging Key Point', 'Logging Point History']
        if 1 <= GestureModel.mode <= 2:
            cv.putText(image, "MODE:" + mode_string[GestureModel.mode - 1], (10, 90),
                    cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
            if 0 <= GestureModel.number <= 9:
                cv.putText(image, "NUM:" + str(GestureModel.number), (10, 110),
                        cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
        

    