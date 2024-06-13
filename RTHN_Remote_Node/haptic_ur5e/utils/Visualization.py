#!/usr/bin/env python3
import cv2 as cv
import copy
from collections import deque

class Visualization:

    def __init__(self, args):
        self.cap = cv.VideoCapture(args.device) #"/dev/video6") #args.device)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, args.width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, args.height)
        self.fps = 0.0
        self.ret = None
        self.image = None
        self.debug_image = None
        self.use_brect = True
        self._start_tick = cv.getTickCount()
        self._freq = 1000.0 / cv.getTickFrequency()
        self._difftimes = deque(maxlen=10)
        
    def capture_image(self):
        self.ret, self.image = self.cap.read()
        if not self.ret:
            return
        #image = cv.resize(self.image, (480, 720)) # ADDED
        self.image = cv.flip(self.image, 1)  # Mirror display
        self.debug_image = copy.deepcopy(self.image)

    def set_writeable(self, bool_state):
        self.image.flags.writeable = bool_state

    def get_fps(self):
        current_tick = cv.getTickCount()
        different_time = (current_tick - self._start_tick) * self._freq
        self._start_tick = current_tick

        self._difftimes.append(different_time)

        fps = 1000.0 / (sum(self._difftimes) / len(self._difftimes))
        fps_rounded = round(fps, 2)

        return fps_rounded

    def draw_point_history(self, point_history, circle_color):
        for index, point in enumerate(point_history):
            if point[0] != 0 and point[1] != 0:
                cv.circle(self.debug_image, (point[0], point[1]), 1 + int(index / 2),
                    circle_color, 2)

    def draw_info(self):
        cv.putText(self.debug_image, "FPS:" + str(self.fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(self.debug_image, "FPS:" + str(self.fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 255, 255), 2, cv.LINE_AA)
        
    def draw_bounding_rect(self, brect):
        if self.use_brect:
            cv.rectangle(self.debug_image, (brect[0], brect[1]), (brect[2], brect[3]), (0, 0, 0), 1)

    def draw_info_text(self, brect, handedness, hand_sign_text, robot_state, mean_landmarks_coords):
        cv.rectangle(self.debug_image, (brect[0], brect[1]), (brect[2], brect[1] - 22), (0, 0, 0), -1)
        info_text = handedness.classification[0].label[0:]
        if hand_sign_text != "":
            info_text = info_text + ':' + hand_sign_text
        cv.putText(self.debug_image, info_text, (brect[0] + 5, brect[1] - 4),
                cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
        if mean_landmarks_coords != [0.0, 0.0, 0.0]:
            cv.putText(self.debug_image, "Robot Active:" + robot_state, (10, 60),
                    cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
            cv.putText(self.debug_image, "Robot Active:" + robot_state, (10, 60),
                    cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                    cv.LINE_AA)
        cv.putText(self.debug_image, "Hand Coordinates:" + str(mean_landmarks_coords), (10, 80),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(self.debug_image, "Hand Coordinates:" + str(mean_landmarks_coords), (10, 80),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)
        
    def draw_landmarks(self, landmark_points, track_type):
        if len(landmark_points) > 0:
            if track_type in [0,1]:
                self.draw_thumb_finger_landmarks(landmark_points)
                self.draw_index_finger_landmarks(landmark_points)
            if track_type == 0:
                self.draw_middle_finger_landmarks(landmark_points)
                self.draw_anular_finger_landmarks(landmark_points)
                self.draw_little_finger_landmarks(landmark_points)
            self.draw_palm_landmarks(landmark_points)

    def draw_thumb_finger_landmarks(self, landmark_points):
        cv.line(self.debug_image, tuple(landmark_points[2]), tuple(landmark_points[3]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[2]), tuple(landmark_points[3]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[3]), tuple(landmark_points[4]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[3]), tuple(landmark_points[4]), (255, 255, 255), 2)

    def draw_index_finger_landmarks(self, landmark_points):
        cv.line(self.debug_image, tuple(landmark_points[5]), tuple(landmark_points[6]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[5]), tuple(landmark_points[6]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[6]), tuple(landmark_points[7]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[6]), tuple(landmark_points[7]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[7]), tuple(landmark_points[8]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[7]), tuple(landmark_points[8]), (255, 255, 255), 2)

    def draw_middle_finger_landmarks(self, landmark_points):
        cv.line(self.debug_image, tuple(landmark_points[9]), tuple(landmark_points[10]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[9]), tuple(landmark_points[10]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[10]), tuple(landmark_points[11]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[10]), tuple(landmark_points[11]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[11]), tuple(landmark_points[12]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[11]), tuple(landmark_points[12]), (255, 255, 255), 2)

    def draw_anular_finger_landmarks(self, landmark_points):
        cv.line(self.debug_image, tuple(landmark_points[13]), tuple(landmark_points[14]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[13]), tuple(landmark_points[14]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[14]), tuple(landmark_points[15]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[14]), tuple(landmark_points[15]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[15]), tuple(landmark_points[16]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[15]), tuple(landmark_points[16]), (255, 255, 255), 2)

    def draw_little_finger_landmarks(self, landmark_points):
        cv.line(self.debug_image, tuple(landmark_points[17]), tuple(landmark_points[18]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[17]), tuple(landmark_points[18]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[18]), tuple(landmark_points[19]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[18]), tuple(landmark_points[19]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[19]), tuple(landmark_points[20]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[19]), tuple(landmark_points[20]), (255, 255, 255), 2)

    def draw_palm_landmarks(self, landmark_points):
        cv.line(self.debug_image, tuple(landmark_points[0]), tuple(landmark_points[1]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[0]), tuple(landmark_points[1]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[1]), tuple(landmark_points[2]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[1]), tuple(landmark_points[2]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[2]), tuple(landmark_points[5]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[2]), tuple(landmark_points[5]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[5]), tuple(landmark_points[9]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[5]), tuple(landmark_points[9]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[9]), tuple(landmark_points[13]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[9]), tuple(landmark_points[13]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[13]), tuple(landmark_points[17]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[13]), tuple(landmark_points[17]), (255, 255, 255), 2)
        cv.line(self.debug_image, tuple(landmark_points[17]), tuple(landmark_points[0]), (0, 0, 0), 6)
        cv.line(self.debug_image, tuple(landmark_points[17]), tuple(landmark_points[0]), (255, 255, 255), 2)

    def draw_thumb_finger_keypoints(self, landmark, index):
        if index == 3:  # 親指：第1関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 4:  # 親指：指先
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    def draw_index_finger_keypoints(self, landmark, index):
        if index == 6:  # 人差指：第2関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 7:  # 人差指：第1関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 8:  # 人差指：指先
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    def draw_middle_finger_keypoints(self, landmark, index):
        if index == 10:  # 中指：第2関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 11:  # 中指：第1関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 12:  # 中指：指先
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    def draw_anular_finger_keypoints(self, landmark, index):
        if index == 14:  # 薬指：第2関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 15:  # 薬指：第1関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 16:  # 薬指：指先
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    def draw_little_finger_keypoints(self, landmark, index):
        if index == 18:  # 小指：第2関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 19:  # 小指：第1関節
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 20:  # 小指：指先
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    def draw_palm_keypoints(self, landmark, index):
        if index == 0:  # 手首1
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 1:  # 手首2
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 2:  # 親指：付け根
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 5:  # 人差指：付け根
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 9:  # 中指：付け根
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 13:  # 薬指：付け根
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 17:  # 小指：付け根
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(self.debug_image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)

    def draw_keypoints(self, landmark_points, track_type):
        if len(landmark_points) > 0:
            for index, landmark in enumerate(landmark_points):
                if track_type in [0,1]:
                    self.draw_thumb_finger_keypoints(landmark, index)
                    self.draw_index_finger_keypoints(landmark, index)
                if track_type == 0:
                    self.draw_middle_finger_keypoints(landmark, index)
                    self.draw_anular_finger_keypoints(landmark, index)
                    self.draw_little_finger_keypoints(landmark, index)
                self.draw_palm_keypoints(landmark, index)
        