#!/usr/bin/env python3
import sys
import rospy
import argparse
import socket
from datetime import datetime
from std_msgs.msg import String
from utils.TrajectoryClient import TrajectoryClient
from utils.Exceptions import *

global args
global trajectory_manager
pub_topic = rospy.Publisher('teleop_robot_arm', String, queue_size=1)

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument( "--communication_mode", type = int, default = 0 )
    parser.add_argument( "--mapping", type = int, default = 0 )
    parser.add_argument( "--duration", type = int, default = 0 )
    parser.add_argument( "--controller", type = int, default = 0 )
    parser.add_argument( "--initial_pos", type = int, default = 4 )       # [0, ... 8] = [up_left, up_center, up_right, mid_left, ..., low_right]
    parser.add_argument( "--oneaxis", type = int, default = -1 )    # [0, 1, 2] = [x, y, z]
    parser.add_argument( "--twoaxis", type = int, default = -1 )    # [0, 1, 2] = [xy, yz, xz]
    parser.add_argument( "--velocity", type = int, default = 0 )
    parser.add_argument( "--acceleration", type = int, default = 0 )
    parser.add_argument( "--precision", type = float, default = -1 )
    parser.add_argument( "--wait_time", type = float, default = -1 )
    parser.add_argument( "--orientation", type = int, default = 0 ) # [0, 1, 2, 3] = [vertical, v-h, h-v, horizontal]
    parser.add_argument( "--mimic", type = int, default = 0 ) # Default: mirror
    parser.add_argument( "--inverted", type = int, default = 0 )
    parser.add_argument( "--bidirectional", type = int, default = 0 )
    args = parser.parse_args()
    print("MIMIC ARGS: ", args.mimic)
    return args

def analyze_inputs(args):
    if args.mapping < 0 or args.mapping > 3:
        raise IncorrectMapping(args.mapping)
    if args.duration < 0 or args.duration > 2:
        raise IncorrectDuration(args.duration)
    if args.controller < 0 or args.controller > 2:
        raise IncorrectController(args.controller)
    if args.initial_pos < 0 or args.initial_pos > 8:
        raise IncorrectInitialPosition(args.initial_pos)
    if args.oneaxis != -1 and args.twoaxis != -1:
        raise IncoherentSpecification()
    if args.oneaxis != -1 and (args.oneaxis < 0 or args.oneaxis > 2):
        raise IncorrectOneAxis(args.oneaxis)
    if args.twoaxis != -1 and (args.twoaxis < 0 or args.twoaxis > 2):
        raise IncorrectTwoAxis(args.twoaxis)
    if args.velocity < 0 or args.velocity > 2:
        raise IncorrectVelocity(args.velocity)
    if args.acceleration < 0 or args.acceleration > 2:
        raise IncorrectAcceleration(args.acceleration)
    if args.precision != -1 and (args.precision < 0.0 or args.precision > 0.12):
        raise IncorrectPrecision(args.precision)
    if args.wait_time != -1 and (args.wait_time < 0.2 or args.wait_time > 2.0):
        raise IncorrectWaitTime(args.wait_time)

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
            if args.bidirectional != 0:
                lst_to_send = []
                lst_to_send.extend(trajectory_manager.human_arm.pose_list)
                lst_to_send.append(msg_received[3]) #Can be changed for LATENCY STUDY
                lst_to_send.append(0) #trajectory_manager.result)
                msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                rospy.loginfo(msg_position)
                pub_topic.publish(msg_position)
    else:
        print("RECIBI 0.0")

def receive_data_update_position(sock_comm):
    trajectory_manager = TrajectoryClient(args)
    comm, addr = sock_comm.accept()
    while True:
        message = comm.recv(1024)
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
                if args.bidirectional != 0:
                    lst_to_send = []
                    lst_to_send.extend(trajectory_manager.human_arm.pose_list)
                    lst_to_send.append(msg_received[3]) #Can be changed for LATENCY STUDY
                    lst_to_send.append(0) #trajectory_manager.result)
                    msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                    #rospy.loginfo(msg_position)
                    #pub_topic.publish(msg_position)
        else:
            print("RECIBI 0.0")

if __name__ == '__main__':
    print("MAIN")
    args = get_args()
    analyze_inputs(args)
    sock_comm = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_comm.bind(('192.168.1.102', 12345))
    sock_comm.listen(5)
    if args.communication_mode == 1:
        pub_topic = rospy.Publisher('teleop_arm_robot', String, queue_size=1)
    rospy.init_node('robot_control', anonymous=True)
    #receive_data_update_position(sock_comm)
    trajectory_manager = TrajectoryClient(args)
    #sock_comm.setblocking(0)
    comm, addr = sock_comm.accept()
    print("Got connection from", addr)
    print("LINK ESTABLISHED. Waiting for commands from Tracking Program ...")
    while True:
        #message = comm.recv(4)
        #msg_length = message.decode()
        #print("RECIBO LARGO: ", msg_length)
        #recv_length = int(msg_length)
        message = comm.recv(55) #comm.recv(recv_length)
        msg_received = message.decode()
        print("ANTES: ", msg_received)
        if len(msg_received) > 1:
            if msg_received[0]=="[" and msg_received[len(msg_received)-1]=="]":
                print("RECIBI: ", msg_received)
                msg_received = get_position_data(msg_received)
                trajectory_manager.human_arm.pose_list = [round(float(msg),3) for msg in msg_received[0:3]]
                if trajectory_manager.human_arm.pose_list != [0.0, 0.0, 0.0]:
                    print("RECIBI ALGO DIFERENTE A 0.0")
                    if trajectory_manager.human_arm.pose_list == [1.0, 1.0, 1.0]:
                        trajectory_manager.on_close(args)
                        if args.communication_mode == 1:
                            lst_to_send_gripper = [1,1,msg_received[3]]
                            msg_gripper = str(lst_to_send_gripper)
                            print("ENVIO: ", msg_gripper)
                            rospy.loginfo(msg_gripper)
                        break
                    else:
                        trajectory_manager.time_received = datetime.strptime(str(msg_received[3][1:(len(msg_received[3])-1)]), "%H:%M:%S:%f")
                        trajectory_manager.move_robot()
                        if args.communication_mode == 1:
                            lst_to_send_gripper = []
                            lst_to_send_gripper.extend(msg_received[4:6])
                            lst_to_send_gripper.append(msg_received[3])
                            msg_gripper = str(lst_to_send_gripper)
                            print("ENVIO: ", msg_gripper)
                            rospy.loginfo(msg_gripper)
                            pub_topic.publish(msg_gripper)
                        if args.bidirectional != 0:
                            lst_to_send = []
                            lst_to_send.extend(trajectory_manager.human_arm.pose_list)
                            lst_to_send.append(msg_received[3]) #Can be changed for LATENCY STUDY
                            lst_to_send.append(0) #trajectory_manager.result)
                            msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                            #rospy.loginfo(msg_position)
                            #pub_topic.publish(msg_position)
                else:
                    print("RECIBI 0.0")
            else:
                print("WRONG FORMAT...")
        else:
            print("NOTHING RECEIVED...")