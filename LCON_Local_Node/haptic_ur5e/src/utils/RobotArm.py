#!/usr/bin/env python3
import math
from utils.TrajectoryLimitator import TrajectoryLimitator
# DEFAULT POSITION: geometry_msgs.Pose(geometry_msgs.Vector3(-0.135, 0.647, 0.550), geometry_msgs.Quaternion(0.9983018, -0.0512478, 0.0101965, -0.0257526 )),

class RobotArm:

    #HUMAN X (COORD 0) is ROBOT Y (COORD 1) :o
    def __init__(self, args):
        self.limitator = TrajectoryLimitator()
        self.configure_start_poses(args.initial_pos)
        self.inverted = args.inverted != 0 # Only Z axis
        print("INVERTED: ", self.inverted)
        self.mimic = args.mimic != 0 # X and Y axis
        print("MIMIC: ", self.mimic)
        self.precision = args.precision
        self.duration_list = [4.0]
        self.velocity = [0.1]
        self.acceleration = [1.0]
        self.configure_orientation(args.orientation)
        self.joint_names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]

    def configure_start_poses(self, initial):
        start_pose = []
        if initial == 0:   # upper-left position
            start_pose = [0.255, 0.700, 0.550]
        elif initial == 1: # upper-centered position
            start_pose = [-0.135, 0.700, 0.550]
        elif initial == 2: # upper-right position
            start_pose = [-0.355, 0.700, 0.550]
        elif initial == 3: # middle-left position
            start_pose = [0.255, 0.700, 0.400]
        elif initial == 5: # middle-right position
            start_pose = [-0.355, 0.700, 0.400]
        elif initial == 6: # lower-left position
            start_pose = [0.255, 0.700, 0.250]
        elif initial == 7: # lower-centered position
            start_pose = [-0.135, 0.700, 0.250]
        elif initial == 8: # lower-right position
            start_pose = [-0.355, 0.700, 0.250]
        else:              # middle-centered position
            start_pose = [-0.135, 0.700, 0.400]
        self.previous_pose = [start_pose]
        self.pose_list = [start_pose]
        self.initial_pose = [start_pose]

    def configure_orientation(self, orientation):
        self.orientation = 0.9983018, -0.0512478, 0.0101965, -0.0257526
        if orientation == 1:
            self.orientation = 0.9357615, 0.2138301, -0.0433734, -0.2770306
        elif orientation == 2:
            self.orientation = 0.8215913, 0.3937745, 0.0586024, -0.4080381
        elif orientation == 3:
            self.orientation = 0.7412093, 0.2729592, 0.4253767, -0.4417655

    def limit_movement(self, pose_element):
        return self.limitator.limit_movement(pose_element)
    
    def is_movement_difference_appropriate(self):
        bool_movement = True
        if self.precision != -1:
            bool_movement = self.get_movement_vector_magnitude() > self.precision #abs(self.pose_list[0][0] - self.previous_pose[0][0]) > tolerance
        #(self.robot_arm.pose_list[len(self.robot_arm.pose_list)-1][0] - pose_element[0]) > tolerance
        return bool_movement
    
    def get_movement_vector_magnitude(self):
        return math.sqrt(sum((self.pose_list[0][i]-self.previous_pose[0][i])**2 for i in range(3)))
    
    def add_velocity_parameters(self, point):
        point.twist.linear.x = self.velocity[0] #0.25 #float("NaN") #self.robot_arm.velocity[i]
        point.twist.linear.y = self.velocity[0] #0.25 #float("NaN")
        point.twist.linear.z = self.velocity[0] #0.25 #float("NaN")
        return point

    def add_acceleration_parameters(self, point):
        point.acceleration.linear.x = self.acceleration[0] #1.2 #float("NaN") #1.2 # self.robot_arm.acceleration[i]
        point.acceleration.linear.y = self.acceleration[0] #1.2 #float("NaN")
        point.acceleration.linear.z = self.acceleration[0] #1.2 #float("NaN")
        point.jerk.linear.x = self.acceleration[0]
        point.jerk.linear.y = self.acceleration[0]
        point.jerk.linear.z = self.acceleration[0]
        return point

    def configure_duration(self):
        if self.is_movement_difference_appropriate(0.01):
            #self.robot_arm.duration_list.append(1.0)
            if self.duration_list[0] < 0.7:
                self.duration_list[0] = 0.7
            if self.duration_list[0] > 1.0:
                self.duration_list[0] = 1.0

    def configure_velocity(self, control_type):
        if control_type == 1:
            velocity = abs(self.pose_list[0][0] - self.previous_pose[0][0])/self.duration_list[0]
        if control_type == 2:
            velocity = 0.25
        else:
            velocity = float("NaN")
        #print("VELOCITY: ", velocity)
        self.velocity[0] = round(velocity,5)

    def configure_acceleration(self, control_type):
        if control_type == 1:
            acceleration = self.velocity[0] / self.duration_list[0]
        elif control_type == 2:
            acceleration = 1.2
        else:
            acceleration = float("NaN")
        #print("ACCELERATION: ", acceleration)
        self.acceleration[0] = round(acceleration,5)

    def configure_duration_model(self):
        magnitude = self.get_movement_vector_magnitude()
        initial_duration = 1 - 2 * magnitude
        if initial_duration < 0.2:
            initial_duration = 0.5
        self.duration_list[0] = initial_duration