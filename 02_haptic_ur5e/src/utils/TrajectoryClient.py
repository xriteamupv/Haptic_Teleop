#!/usr/bin/env python3
import sys
import rospy
from datetime import datetime
from utils.RobotArm import RobotArm
from utils.HumanArm import HumanArm 
from utils.Gripper import Gripper
from utils.Controller import Controller 
from utils.Mapper import Mapper
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

class TrajectoryClient:

    def __init__(self, args):
        self.mapping = args.mapping
        self.oneaxis = args.oneaxis
        self.twoaxis = args.twoaxis
        self.time_received = datetime.now()
        self.current_time = datetime.now()
        self.controller = Controller(args)
        self.human_arm = HumanArm()
        self.robot_arm = RobotArm(args)
        self.gripper = Gripper()
        self.send_cartesian_trajectory()

    def configure_oneaxis(self, pose):
        geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(pose[0], pose[1], pose[2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        if self.oneaxis == 0:
            print("POSE: ", pose)
            print("INITIAL POSE: ", self.robot_arm.initial_pose)
            geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(pose[0], self.robot_arm.initial_pose[0][1], self.robot_arm.initial_pose[0][2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        elif self.oneaxis == 1:
            geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(self.robot_arm.initial_pose[0][0], pose[1], self.robot_arm.initial_pose[0][2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        elif self.oneaxis == 2:
            geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(self.robot_arm.initial_pose[0][0], self.robot_arm.initial_pose[0][1], pose[2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        return geometry_pose
    
    def configure_twoaxis(self, pose):
        geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(pose[0], pose[1], pose[2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        if self.twoaxis == 0:
            geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(pose[0], pose[1], self.robot_arm.initial_pose[0][2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        elif self.twoaxis == 1:
            geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(self.robot_arm.initial_pose[0][0], pose[1], pose[2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        elif self.twoaxis == 2:
            geometry_pose = geometry_msgs.Pose(
                    geometry_msgs.Vector3(pose[0], self.robot_arm.initial_pose[0][1], pose[2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 ))
        return geometry_pose

    def configure_trajectory_pose(self):
        pose_element = []
        for i, pose in enumerate(self.robot_arm.pose_list):
            if self.oneaxis != None:
                pose_element.append(self.configure_oneaxis(pose))
            elif self.twoaxis != None:
                pose_element.append(self.configure_twoaxis(pose))
            else:
                pose_element.append(geometry_msgs.Pose(
                    geometry_msgs.Vector3(pose[0], pose[1], pose[2]), 
                    geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 )))
        return pose_element
    
    def configure_trajectory_goal(self, pose_element):
        goal = FollowCartesianTrajectoryGoal()
        for i, pose in enumerate(pose_element):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(self.robot_arm.duration_list[i])
            if self.controller.control_velocity != 0.0:
                point = self.robot_arm.add_velocity_parameters(point)
            if self.controller.control_acceleration != 0.0:
                point = self.robot_arm.add_acceleration_parameters(point)
            goal.trajectory.points.append(point)
        return goal
                
    def send_cartesian_trajectory(self):
        
        pose_element = self.configure_trajectory_pose()
        goal = self.configure_trajectory_goal(pose_element)

        print("HUMAN ARM: ", self.human_arm.pose_list)
        print("ROBOT ARM: ", self.robot_arm.pose_list)
        print("DURATION: ", self.robot_arm.duration_list)
        print("GOAL: ",goal.trajectory.points)
        
        self.controller.execute_goal(goal)

    def get_movement_vector_magnitude(self):
        return self.robot_arm.get_movement_vector_magnitude()

    def limit_movement(self, pose_element):
        return self.robot_arm.limit_movement(pose_element)
    
    def configure_duration_arm(self):
        if self.controller.control_duration == 1:
            time_delta = self.time_received - self.current_time
            time_diff_sec = round(float(time_delta.microseconds)*0.000001, 3) 
            self.robot_arm.duration_list[0] = time_diff_sec
        else:
            self.robot_arm.configure_duration_model() # default
        self.current_time = self.time_received

    def configure_pose_arm(self):
        if self.mapping == 1:
            pose_element = Mapper.focused_linear_mapping_arm(self.human_arm, self.robot_arm)
        elif self.mapping == 2:
            pose_element = Mapper.multi_focused_linear_mapping_arm(self.human_arm, self.robot_arm)
        elif self.mapping == 3:
            pose_element = Mapper.non_linear_mapping_arm(self.human_arm, self.robot_arm)
        else:
            pose_element = Mapper.linear_mapping_arm(self.human_arm, self.robot_arm) #default
        self.robot_arm.previous_pose = self.robot_arm.pose_list
        self.robot_arm.pose_list = [[round(pose_element[0],3), round(pose_element[1],3), round(pose_element[2],3)]]

    def configure_additional_parameters(self):
        if self.controller.control_velocity != 0:
            self.robot_arm.configure_velocity(self.controller.control_velocity)
        if self.controller.control_acceleration != 0:
            self.robot_arm.configure_acceleration(self.controller.control_acceleration)

    def move_robot(self):
        self.configure_duration_arm()
        self.configure_pose_arm()
        #self.robot_arm.configure_duration()
        self.configure_additional_parameters()
        if self.robot_arm.is_movement_difference_appropriate():
            self.send_cartesian_trajectory()

    def on_close(self, args):
        self.robot_arm = RobotArm(args)
        self.send_cartesian_trajectory()