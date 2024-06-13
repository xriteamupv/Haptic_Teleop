#!/usr/bin/env python3
import sys
import rospy
import socket
import argparse
from datetime import datetime
from std_msgs.msg import String
from utils.TrajectoryClient import TrajectoryClient

global args
global trajectory_manager
pub_topic = rospy.Publisher('teleop_robot_arm', String, queue_size=1)

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=int, default=0)
    parser.add_argument("--duration", type=int, default=0)
    parser.add_argument("--controller", type=int, default=0)
    parser.add_argument("--initial", type=int, default=4)       # [0, ... 8] = [up_left, up_center, up_right, mid_left, ..., low_right]
    parser.add_argument("--oneaxis", type=int, default=None)    # [0, 1, 2] = [x, y, z] CHECKED
    parser.add_argument("--twoaxis", type=int, default=None)    # [0, 1, 2] = [xy, yz, xz] CHECKED
    parser.add_argument("--velocity", type=int, default=0)
    parser.add_argument("--acceleration", type=int, default=0)
    parser.add_argument("--precision", type=float, default=None)
    parser.add_argument("--wait", type=float, default=None)
    parser.add_argument("--mimic", type=bool, default=False) # Default: mirror CHECKED
    parser.add_argument("--inverted", type=bool, default=False) # CHECKED
    parser.add_argument("--bidirectional", type=bool, default=False) # CHECKED
    args = parser.parse_args()
    return args

def correct_input_args(args):
    #bool_mode_correct = not (args.mimic and args.mirror)
    bool_axis_correct = not (args.oneaxis != None and args.twoaxis != None)
    return bool_axis_correct #bool_mode_correct and bool_axis_correct

def get_position_data(str_received):
    str_pos = str_received.split("[")[1]
    str_pos = str_pos.split("]")[0]
    str_pos = str_pos.split(", ")
    return str_pos

def update_position(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    msg_received = get_position_data(data.data)
    trajectory_manager.human_arm.pose_list = [round(float(msg),3) for msg in msg_received[0:3]]
    if trajectory_manager.human_arm.pose_list != [0.0, 0.0, 0.0]:
        print("RECIBI ALGO DIFERENTE A 0.0")
        if trajectory_manager.human_arm.pose_list == [1.0, 1.0, 1.0]:
            trajectory_manager.on_close(args)
        else:
            trajectory_manager.time_received = datetime.strptime(str(msg_received[3][1:(len(msg_received[3])-1)]), "%H:%M:%S:%f")
            trajectory_manager.move_robot()
            if args.bidirectional:
                lst_to_send = []
                lst_to_send.extend(trajectory_manager.human_arm.pose_list)
                lst_to_send.append(msg_received[3]) #Can be changed for LATENCY STUDY
                lst_to_send.append(0) #trajectory_manager.result)
                msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                rospy.loginfo(msg_position)
                pub_topic.publish(msg_position)
    else:
        print("RECIBI 0.0")

if __name__ == '__main__':
    print("MAIN")
    args = get_args()
    if correct_input_args(args):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('192.168.1.102', 12345))
        s.listen(5)
        rospy.init_node('robot_control', anonymous=True)
        trajectory_manager = TrajectoryClient(args)
        c, addr = s.accept()
        print('Got connection from', addr)
        while True:
            message = c.recv(1024)
            msg_received = get_position_data(message.decode())
            trajectory_manager.human_arm.pose_list = [round(float(msg),3) for msg in msg_received[0:3]]
            if trajectory_manager.human_arm.pose_list != [0.0, 0.0, 0.0]:
                print("RECIBI ALGO DIFERENTE A 0.0")
                if trajectory_manager.human_arm.pose_list == [1.0, 1.0, 1.0]:
                    trajectory_manager.on_close(args)
                    break
                else:
                    trajectory_manager.time_received = datetime.strptime(str(msg_received[3][1:(len(msg_received[3])-1)]), "%H:%M:%S:%f")
                    trajectory_manager.move_robot()
                    if args.bidirectional:
                        lst_to_send = []
                        lst_to_send.extend(trajectory_manager.human_arm.pose_list)
                        lst_to_send.append(msg_received[3]) #Can be changed for LATENCY STUDY
                        lst_to_send.append(0) #trajectory_manager.result)
                        msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                        rospy.loginfo(msg_position)
                        pub_topic.publish(msg_position)
            else:
                print("RECIBI 0.0")
        #rospy.Subscriber("teleop_arm_robot", String, callback=update_position, queue_size=1)
        #rospy.spin()