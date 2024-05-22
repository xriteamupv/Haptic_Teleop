#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv
import copy
import argparse
import itertools
from collections import Counter
from collections import deque

import cv2 as cv
import numpy as np
import mediapipe as mp

#import haptic_u5e as hp
#from haptic_u5e import * 
from utils import CvFpsCalc
from model import KeyPointClassifier
from model import PointHistoryClassifier

import sys
import rospy
import actionlib
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

import datetime

#from google.protobuf.json_format import MessageToDict

saturation = 0
brightness = 0
hue = 0

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]
num_controller = 1

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

class RobotArm:

    #HUMAN X (COORD 0) is ROBOT Y (COORD 1) :o
    def __init__(self):
        self.min_limits = [-0.400, 0.300, 0.300] 
        self.max_limits = [0.400, 0.700, 0.600] #ADELANTE
        self.pose_list = [[-0.135, 0.647, 0.550]]
        # geometry_msgs.Pose(geometry_msgs.Vector3(-0.135, 0.647, 0.550), geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 )),
        self.duration_list = [4.0]

class HumanArm:

    def __init__(self):
        self.min_limits = [0, 0, -0.01]
        self.max_limits = [1, 1, -0.04]
        self.pose_list = [0.0, 0.0, 0.0]
        #self.arm_tracking = CameraTracking()

class TrajectoryClient:

    def __init__(self):
        rospy.init_node("arm_control_tracking")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        #self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[num_controller]

        self.human_arm = HumanArm()
        self.robot_arm = RobotArm()

        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(1)
        if not self.trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)
        self.send_cartesian_trajectory()

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def is_movement_difference_appropriate(self, pose_element, tolerance):
        print("POSE_LIST: ", self.robot_arm.pose_list)
        print("POSE_ELEMENT: ", pose_element)
        #bool_movement_x = False
        #if self.robot_arm.pose_list != [[]]:
        print("COMP. ROBOT: ", self.robot_arm.pose_list[len(self.robot_arm.pose_list)-1][0])
        print("COMP. POSE: ", pose_element[0])
        bool_movement_x = abs(self.robot_arm.pose_list[0][0] - pose_element[0]) > tolerance
        #(self.robot_arm.pose_list[len(self.robot_arm.pose_list)-1][0] - pose_element[0]) > tolerance
        return bool_movement_x
                
    def send_cartesian_trajectory(self):
        
        goal = FollowCartesianTrajectoryGoal()
        pose_element = []
        for i, pose in enumerate(self.robot_arm.pose_list):
            pose_element.append(geometry_msgs.Pose(
                geometry_msgs.Vector3(pose[0], 0.647, 0.55), 
                geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 )))

        for i, pose in enumerate(pose_element):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(self.robot_arm.duration_list[i])
            goal.trajectory.points.append(point)

        print("HUMAN ARM: ", self.human_arm.pose_list)
        print("ROBOT ARM: ", self.robot_arm.pose_list)
        #print("GOAL: ", goal)
        rospy.loginfo(
            "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
        )
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        #result = trajectory_client.get_result()
        #rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    # LINEAR MAPPING (ver otros mapeos)
    def linear_mapping_arm(self):
        pose_element = [0, 0, 0]
        proportion_element = [0, 0, 0]
        #for i in range(len(pose_element)):
        
        # EJE HUMAN X
        proportion_element[0] = (self.human_arm.max_limits[0]-self.human_arm.min_limits[0])*(self.robot_arm.max_limits[0]-self.robot_arm.min_limits[0])
        pose_element[0] = self.robot_arm.min_limits[0] + proportion_element[0]*self.human_arm.pose_list[0]

        # EJE HUMAN Y
        proportion_element[1] = (self.human_arm.max_limits[1]-self.human_arm.min_limits[1])*(self.robot_arm.max_limits[1]-self.robot_arm.min_limits[1])
        #pose_element[1] = self.robot_arm.max_limits[1] - proportion_element[1]*abs(self.robot_arm.max_limits[1]-self.robot_arm.min_limits[1])
        pose_element[1] = self.robot_arm.min_limits[1] + proportion_element[1]*self.human_arm.pose_list[1]

        if pose_element[0] < self.robot_arm.min_limits[0]:
            pose_element[0] = self.robot_arm.min_limits[0]
        if pose_element[0] > self.robot_arm.max_limits[0]:
            pose_element[0] = self.robot_arm.max_limits[0]

        print("BEFORE MOVEMENT ANALYSIS")
        # CONTROL VARIACION VALORES CONSECUTIVOS
        if self.is_movement_difference_appropriate(pose_element, 0.1):
            print("MOVEMENT APPROPRIATE")
            self.robot_arm.duration_list.append(0.1)
            self.robot_arm.pose_list = [[-round(pose_element[0],3), 0.647, 0.550]] #.append([-pose_element[0], 0.647, 0.550])
            #if len(self.robot_arm.pose_list) > 4:
            #    self.robot_arm.pose_list.pop(0)

        #self.robot_arm.pose_list = [
        #    geometry_msgs.Pose(
        #        geometry_msgs.Vector3(pose_element[1], pose_element[0], pose_element[2]), geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 )
        #    ),
        #]

        #self.robot_arm.duration_list = [7.0]

    def on_close(self):
        self.robot_arm = RobotArm()
        self.send_cartesian_trajectory()

class HueHelper:
    def __init__(self):
        self.image = np.zeros((960, 540, 3), dtype = np.uint8)
        self.debug_image = None

    def apply_hue_offset(self):# 0 is no change; 0<=huechange<=180
            # convert img to hsv
            #image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img_hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
            h = img_hsv[:,:,0]
            s = img_hsv[:,:,1]
            v = img_hsv[:,:,2]
            # shift the hue
            # cv2 will clip automatically to avoid color wrap-around
            img_hsv = cv.add(h, hue)
            img_hsv = cv.add(s, saturation)
            
            lim = 255 - brightness
            v[v > lim] = 255
            v[v <= lim] += brightness
            # combine new hue with s and v
            img_hsv = cv.merge([img_hsv,s,v])
            #img_hsv = cv2.convertScaleAbs(img_hsv, beta=5)
            # convert from HSV to BGR
            return cv.cvtColor(img_hsv, cv.COLOR_HSV2BGR)

    def on_trackbar_change(self, trackbar_hue_offset):
        global hue
        hue = trackbar_hue_offset
        self.image = self.apply_hue_offset()
        print("H C B = ", hue, " ", saturation, " ", brightness)
        #cv.imshow("Recognition", self.image)

    def on_trackbar_changeA(self, trackbar_contrast):
        global saturation
        saturation = trackbar_contrast
        self.image = self.apply_hue_offset()
        print("H C B = ", hue, " ", saturation, " ", brightness)
        #cv.imshow("Recognition", self.image)

    def on_trackbar_changeB(self, trackbar_brightness):
        global brightness
        brightness = trackbar_brightness
        self.image = self.apply_hue_offset()
        print("H C B = ", hue, " ", saturation, " ", brightness)
        #cv.imshow("Recognition", self.image)

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument('--use_static_image_mode', action='store_true')
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence',
                        type=float,
                        default=0.7)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence',
                        type=int,
                        default=0.5)

    args = parser.parse_args()

    return args


def main():
    # Argument parsing #################################################################
    args = get_args()
    helper = HueHelper()
    trajectory_manager = TrajectoryClient()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    use_static_image_mode = args.use_static_image_mode
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    use_brect = True

    # Camera preparation ###############################################################
    cap = cv.VideoCapture(cap_device)
    #cap = cv.VideoCapture("IMG_3701.MOV")
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)
    
    point_reference = [0.5, 0.5, 0.01] #[0.5, 0.15, 0.01] #[0.5, 0.5, 0.01]
    tolerance = [0.5, 0.5, 0.06] #[0.15, 0.15, 0.06]

    # We need to set resolutions. 
    # so, convert them from float to integer. 
    frame_width = int(cap.get(3)) 
    frame_height = int(cap.get(4)) 
    
    #size = (cap_width, cap_height) 
    
    # Below VideoWriter object will create 
    # a frame of above defined The output  
    # is stored in 'filename.avi' file. 
    #video_out = cv.VideoWriter('10052024_gloves_diff_grips3.mp4', cv.VideoWriter_fourcc(*"mp4v"), 20, (frame_width, frame_height)) 
    #cv.VideoWriter_fourcc(*"mp4v"), 
    # Model load #############################################################
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=use_static_image_mode,
        max_num_hands=1,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    keypoint_classifier = KeyPointClassifier()

    point_history_classifier = PointHistoryClassifier()

    # Read labels ###########################################################
    with open('/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/keypoint_classifier/keypoint_classifier_label.csv',
              encoding='utf-8-sig') as f:
        keypoint_classifier_labels = csv.reader(f)
        #print("LABELS: ", keypoint_classifier_labels)
        keypoint_classifier_labels = [
            row[0] for row in keypoint_classifier_labels
        ]
        #print("LABELS 2: ", keypoint_classifier_labels)
    with open(
            '/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/point_history_classifier/point_history_classifier_label.csv',
            encoding='utf-8-sig') as f:
        point_history_classifier_labels = csv.reader(f)
        point_history_classifier_labels = [
            row[0] for row in point_history_classifier_labels
        ]

    # FPS Measurement ########################################################
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Coordinate history #################################################################
    history_length = 16
    point_history = deque(maxlen=history_length)

    # Finger gesture history ################################################
    finger_gesture_history = deque(maxlen=history_length)

    #  ########################################################################
    mode = 0

    hand_detected = False
    cuenta_robot = 1
    #hue_offset = 70
    #b = 0

    #helper.on_trackbar_change(hue)
    #helper.on_trackbar_changeA(saturation)
    #helper.on_trackbar_changeB(brightness)
    # Hue range is 0-179: https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
    #cv.createTrackbar("Hue Offset", "Recognition", 0, 179, helper.on_trackbar_change)
    #cv.createTrackbar("Saturation", "Recognition", 0, 200, helper.on_trackbar_changeA)
    #cv.createTrackbar("Brightness", "Recognition", 0, 200, helper.on_trackbar_changeB)
    mean_landmarks_coords = [0.0, 0.0, 0.0]
    robot_state = "Stopped"
    cuenta_state = 20
    first_time = datetime.datetime.now()

    while True:
        fps = cvFpsCalc.get()
        second_time = datetime.datetime.now()
        time_delta = second_time-first_time
        print("DIFERENCIA TIEMPO: "+str(time_delta.microseconds))
        first_time = datetime.datetime.now()

        # Process Key (ESC: end) #################################################
        key = cv.waitKey(10)
        if key == 27:  # ESC
            break
        number, mode = select_mode(key, mode)

        # Camera capture #####################################################
        ret, image = cap.read()
        if not ret:
            break
        #image = cv.resize(image, (480, 720)) # ADDED
        helper.image = cv.flip(image, 1)  # Mirror display
        helper.debug_image = copy.deepcopy(helper.image)

        # Detection implementation #############################################################
        #helper.image = cv.cvtColor(helper.image, cv.COLOR_BGR2RGB)

        #helper.debug_image = helper.image

        helper.image.flags.writeable = False
        results = hands.process(helper.image)
        helper.image.flags.writeable = True

        bool_near_reference = is_hand_near_reference(mean_landmarks_coords, point_reference, tolerance)

        #  ####################################################################
        if results.multi_hand_landmarks is not None:
            #hand_detected = True
            #print("HANDEDNESS: ", results.multi_handedness[0]["label"])
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):
                
                #handedness_dict = MessageToDict(handedness)
                #detected_hand = handedness_dict['classification'][0]['label']
                #print("HANDEDNESS: ", handedness_dict['classification'][0]['label'])
                #if detected_hand == "Right" or detected_hand == "Left":
                # Bounding box calculation
                brect = calc_bounding_rect(helper.debug_image, hand_landmarks)
                #print("LANDMARKS: ", hand_landmarks)
                # Landmark calculation
                landmark_list, landmark_coord = calc_landmark_list(helper.debug_image, hand_landmarks)
                all_coord_x = [coord[0] for coord in landmark_coord]
                all_coord_y = [coord[1] for coord in landmark_coord]
                all_coord_z = [coord[2] for coord in landmark_coord]
                len_coord = len(all_coord_x)
                if(len_coord == 0):
                    len_coord = 0.000001
                #print("MEAN X = ", sum(all_coord_x)/len_coord)
                #print("MEAN Y = ", sum(all_coord_y)/len_coord)
                #print("MEAN Z = ", sum(all_coord_z)/len_coord)
                mean_landmarks_coords[0] = round(sum(all_coord_x)/len_coord,3)
                mean_landmarks_coords[1] = round(sum(all_coord_y)/len_coord,3)
                mean_landmarks_coords[2] = round(sum(all_coord_z)/len_coord,3)

                all_land_x = [landmark[0] for landmark in landmark_list]
                all_land_y = [landmark[1] for landmark in landmark_list]
                #all_land_z = [coord[2] for coord in landmark_list]
                len_land = len(all_land_x)
                if(len_land == 0):
                    len_land = 0.000001

                mean_landmarks_list = [0, 0]
                mean_landmarks_list[0] = int(sum(all_land_x)/len_land)
                mean_landmarks_list[1] = int(sum(all_land_y)/len_land)

                # Conversion to relative coordinates / normalized coordinates
                pre_processed_landmark_list = pre_process_landmark(
                    landmark_list)
                pre_processed_point_history_list = pre_process_point_history(
                    helper.debug_image, point_history)
                # Write to the dataset file
                logging_csv(number, mode, pre_processed_landmark_list,
                            pre_processed_point_history_list)

                # Hand sign classification
                hand_sign_id = keypoint_classifier(pre_processed_landmark_list)
                #if hand_sign_id == 2:  # Open gesture
                #print("MEAN LANDMARK", mean_landmarks_list)
                point_history.append(mean_landmarks_list)
                #print("LANDMARK", landmark_list[8])
                #else:
                #    point_history.append([0, 0])

                cuenta_state = cuenta_state + 1
                if hand_sign_id == 3 and bool_near_reference and cuenta_state > 20:
                    if robot_state == "Stopped":
                        robot_state = "Running"
                    else:
                        robot_state = "Stopped"
                    cuenta_state = 0

                # Finger gesture classification
                finger_gesture_id = 0
                point_history_len = len(pre_processed_point_history_list)
                if point_history_len == (history_length * 2):
                    finger_gesture_id = point_history_classifier(
                        pre_processed_point_history_list)

                # Calculates the gesture IDs in the latest detection
                finger_gesture_history.append(finger_gesture_id)
                most_common_fg_id = Counter(
                    finger_gesture_history).most_common()

                # Drawing part
                helper.debug_image = draw_bounding_rect(use_brect, helper.debug_image, brect)
                helper.debug_image = draw_landmarks(helper.debug_image, landmark_list)

                helper.debug_image = draw_info_text(
                    helper.debug_image,
                    brect,
                    handedness,
                    keypoint_classifier_labels[hand_sign_id],
                    robot_state,
                    #point_history_classifier_labels[most_common_fg_id[0][0]],
                    mean_landmarks_coords,
                )
        else:
            #hand_detected = False
            point_history.append([0, 0])

        if(bool_near_reference):
            #print("NEAR TOLERANCE")
            helper.debug_image = draw_point_history(helper.debug_image, point_history, (152, 251, 152))
        else:
            helper.debug_image = draw_point_history(helper.debug_image, point_history, (0,0,255))
        helper.debug_image = draw_info(helper.debug_image, fps, mode, number)

        #video_out.write(helper.debug_image)
        # Screen reflection #############################################################
        cv.imshow('Hand Gesture Recognition', helper.debug_image)
        
        if robot_state == "Running":
            cuenta_robot = cuenta_robot + 1
            trajectory_manager.human_arm.pose_list = mean_landmarks_coords
            trajectory_manager.linear_mapping_arm()
            if (cuenta_robot % 4 == 0):
                #if(len(trajectory_manager.robot_arm.pose_list[0]) > 0):
                #print("LUEGO IF POSE_LIST: ", trajectory_manager.robot_arm.pose_list)
                #print("LUEGO IF: ", len(trajectory_manager.robot_arm.pose_list) > 0)
                #trajectory_manager.robot_arm.pose_list = [[0.300, 0.647, 0.55]]
                trajectory_manager.send_cartesian_trajectory()
        #trajectory_manager.robot_arm.pose_list = [[]]

    cap.release()
    cv.destroyAllWindows()
    #trajectory_manager.robot_arm.pose_list = [[0.300, 0.647, 0.55]]
    #trajectory_manager.send_cartesian_trajectory()
    trajectory_manager.on_close()


def select_mode(key, mode):
    number = -1
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if 97 <= key <= 103:  # a ~ g
        number = key - 87
        print("Number: ", number)
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    if key == 104:  # h
        mode = 2
    return number, mode


def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_coord = []
    landmark_point = []

    # Keypoint
    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        landmark_z = landmark.z
        #print("LANDMARK X:", landmark_x)
        #print("LANDMARK Y:", landmark_y)
        #print("LANDMARK Z:", landmark_z)

        landmark_point.append([landmark_x, landmark_y])
        landmark_coord.append([landmark.x, landmark.y, landmark.z])

    return landmark_point, landmark_coord


def pre_process_landmark(landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
        if index == 0:
            base_x, base_y = landmark_point[0], landmark_point[1]

        temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
        temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

    # Convert to a one-dimensional list
    temp_landmark_list = list(
        itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


def pre_process_point_history(image, point_history):
    image_width, image_height = image.shape[1], image.shape[0]

    temp_point_history = copy.deepcopy(point_history)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, point in enumerate(temp_point_history):
        if index == 0:
            base_x, base_y = point[0], point[1]

        temp_point_history[index][0] = (temp_point_history[index][0] -
                                        base_x) / image_width
        temp_point_history[index][1] = (temp_point_history[index][1] -
                                        base_y) / image_height

    # Convert to a one-dimensional list
    temp_point_history = list(
        itertools.chain.from_iterable(temp_point_history))

    return temp_point_history


def logging_csv(number, mode, landmark_list, point_history_list):
    if mode == 0:
        pass
    if mode == 1 and (0 <= number <= 16):
        csv_path = '/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/keypoint_classifier/keypoint.csv'
        print("Logging: Num ", number, " Mode ", mode)
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *landmark_list])
    if mode == 2 and (0 <= number <= 9):
        csv_path = '/home/ros-iteam/catkin_ws/src/haptic_u5e/src/model/point_history_classifier/point_history.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *point_history_list])
    return


def draw_landmarks(image, landmark_point):
    if len(landmark_point) > 0:
        # Thumb
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (255, 255, 255), 2)

        # Index finger
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (255, 255, 255), 2)

        # Middle finger
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (255, 255, 255), 2)

        # Ring finger
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (255, 255, 255), 2)

        # Little finger
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (255, 255, 255), 2)

        # Palm
        cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (255, 255, 255), 2)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (0, 0, 0), 6)
        cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (255, 255, 255), 2)

    # Key Points
    for index, landmark in enumerate(landmark_point):
        if index == 0:  # 手首1
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 1:  # 手首2
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 2:  # 親指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 3:  # 親指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 4:  # 親指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 5:  # 人差指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 6:  # 人差指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 7:  # 人差指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 8:  # 人差指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 9:  # 中指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 10:  # 中指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 11:  # 中指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 12:  # 中指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 13:  # 薬指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 14:  # 薬指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 15:  # 薬指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 16:  # 薬指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 17:  # 小指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 18:  # 小指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 19:  # 小指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 20:  # 小指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    return image


def draw_bounding_rect(use_brect, image, brect):
    if use_brect:
        # Outer rectangle
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 0, 0), 1)

    return image


def draw_info_text(image, brect, handedness, hand_sign_text,
                   robot_state, mean_landmarks_coords):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                 (0, 0, 0), -1)

    info_text = handedness.classification[0].label[0:]
    if hand_sign_text != "":
        info_text = info_text + ':' + hand_sign_text
    cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

    if mean_landmarks_coords != [0.0, 0.0, 0.0]:
        cv.putText(image, "Robot Active:" + robot_state, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, "Robot Active:" + robot_state, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                   cv.LINE_AA)

    #if finger_gesture_text != "":
    #    cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
    #               cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
    #    cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
    #               cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
    #               cv.LINE_AA)
        
    cv.putText(image, "Hand Coordinates:" + str(mean_landmarks_coords), (10, 80),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, "Hand Coordinates:" + str(mean_landmarks_coords), (10, 80),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv.LINE_AA)

    return image

def is_hand_near_reference(point, point_reference, tolerance):
    #print("CHECK: ", [abs(point[i] - point_reference[i]) for i in [0,1,2]])
    is_near_reference = [abs(point[i] - point_reference[i]) <= tolerance[i] for i in [0,1,2]]
    #print("CHECK: ", is_near_reference)
    return all(is_near_reference)

def draw_point_history(image, point_history, circle_color):
    for index, point in enumerate(point_history):
        if point[0] != 0 and point[1] != 0:
            cv.circle(image, (point[0], point[1]), 1 + int(index / 2),
                circle_color, 2)
    return image


def draw_info(image, fps, mode, number):
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255, 255, 255), 2, cv.LINE_AA)

    mode_string = ['Logging Key Point', 'Logging Point History']
    if 1 <= mode <= 2:
        cv.putText(image, "MODE:" + mode_string[mode - 1], (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                   cv.LINE_AA)
        if 0 <= number <= 9:
            cv.putText(image, "NUM:" + str(number), (10, 110),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                       cv.LINE_AA)
    return image


if __name__ == '__main__':
    main()
