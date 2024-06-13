#!/usr/bin/env python3
import sys
import rospy
import argparse
import socket
from std_msgs.msg import String, Float32
from utils.GripClient import GripClient
from utils.Exceptions import *

global args
global gripper_manager
global do_update
global msg_recv_robot
pub_topic = rospy.Publisher('teleop_robot_gripper', String, queue_size=1)

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument( "--communication_mode", type = int, default = 0 )
    parser.add_argument( "--max_width", type = float, default = 100.0 )
    parser.add_argument( "--max_force", type = float, default = 40.0 )
    parser.add_argument( "--start_width", type = float, default = 100.0 )
    parser.add_argument( "--width_tolerance", type = float, default = 2.0 )
    parser.add_argument( "--force_tolerance", type = float, default = 2.0 )
    parser.add_argument( "--max_duration", type = float, default = 60.0 ) # mseconds
    parser.add_argument( "--time_max_width", type = float, default = 5.0 ) # seconds
    parser.add_argument( "--static_force", type = float, default = 39.0 ) # Netwons
    parser.add_argument( "--width_model", type = int, default = 0 )
    parser.add_argument( "--force_model", type = int, default = 0 )
    parser.add_argument( "--delay_model", type = int, default = 1 )
    parser.add_argument( "--grip_levels", type = int, default = 4 ) # CHECK 3 or 4
    parser.add_argument( "--bidirectional", type = int, default = 0 ) #PENDING
    args = parser.parse_args()
    return args

def analyze_inputs(args):
    c = 1 # PENDING

def get_position_data(str_received):
    str_pos = str_received.split("[")[1]
    str_pos = str_pos.split("]")[0]
    str_pos = str_pos.split(", ")
    return str_pos

def update_gripper_state(data):
    #rospy.loginfo(rospy.get_caller_id() + " GRIPPER: I heard %s", data.data)
    rec_width = round(float(data.data), 3)
    if gripper_manager.is_width_correct(rec_width):
        gripper_manager.update_width(rec_width)
        gripper_manager.update_detection()

def update_robot_frame(data):
    #rospy.loginfo(rospy.get_caller_id() + " ROBOT: I heard %s", data.data)
    global msg_recv_robot
    msg_recv_robot = str(data.data)

def gripper_action(communication_mode, msg_recv, sock_comm_glove):
    break_loop = False
    msg_received = get_position_data(msg_recv)
    if communication_mode == 1:
        msg_received = [msg[1:(len(msg)-1)] for msg in msg_received]
    gripper_control = [int(msg) for msg in msg_received[0:2]]
    gripper_manager.update_time_received(msg_received)
    gripper_manager.process_grip_level(int(msg_received[0])) 
    gripper_manager.configure_force(round(args.static_force,3))
    fingers = int(msg_received[1])
    if gripper_control == [1, 1]:
            print("CERRANDO...")
            gripper_manager.on_open_or_close()
            msg_send = [1.0, 1.0, 0]
            msg_send.append(gripper_manager.event_manager.time_received.strftime("%H:%M:%S:%f"))
            msg_send_glove = str(msg_send)
            sock_comm_glove.send(msg_send_glove.encode())
            break_loop = True
    elif gripper_manager.is_movement_detected() or fingers == 6:
        gripper_manager.gripper_operation()
        msg_send = []
        msg_send.append(gripper_manager.get_width_variation())
        msg_send.append(gripper_manager.get_grip_force())
        msg_send.append(fingers) # FINGERS
        msg_send.append(gripper_manager.event_manager.time_received.strftime("%H:%M:%S:%f"))
        msg_send_glove = str(msg_send)
        sock_comm_glove.send(msg_send_glove.encode())
        if args.bidirectional != 0:
            lst_to_send = []
            lst_to_send.extend(msg_received[0:3])
            lst_to_send.append(0) #gripper_manager.result)
            msg_position = str(lst_to_send)
            #rospy.loginfo(msg_position)
            #pub_topic.publish(msg_position)
    else:
        print("NO MOVEMENT DETECTED")
    return break_loop

if __name__ == '__main__':
    print("MAIN")
    do_update = True
    msg_recv_robot = ""
    args = get_args()
    analyze_inputs(args)
    if args.communication_mode == 0:
        sock_comm_track = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_comm_track.bind(('192.168.1.102', 23456))
        sock_comm_track.listen(5)
        comm, addr = sock_comm_track.accept()
        print("Got connection from", addr)
        print("LINK ESTABLISHED. Waiting for commands from Tracking Program ...")
    elif args.communication_mode == 1:
        pub_sub_robot = rospy.Subscriber("teleop_arm_robot", String, callback=update_robot_frame, queue_size=1)
    sock_comm_glove = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_comm_glove.connect(('10.45.26.2', 34567))
    rospy.init_node('gripper_control', anonymous=True)
    gripper_manager = GripClient(args)
    pub_sub_gripper = rospy.Subscriber("/gripper_controller/get_pos", Float32, callback=update_gripper_state, queue_size=1)
    while True:
        break_loop = False
        if args.communication_mode == 0:
            message = comm.recv(55)
            msg_recv = message.decode()
            if len(msg_recv) > 1:
                if msg_recv[0]=="[" and msg_recv[len(msg_recv)-1]=="]":
                    print("RECIBI: ", msg_recv)
                    break_loop = gripper_action(args.communication_mode, msg_recv, sock_comm_glove)
        elif args.communication_mode == 1:
            if len(msg_recv_robot) > 1:
                if msg_recv_robot[0]=="[" and msg_recv_robot[len(msg_recv_robot)-1]=="]":
                    print("RECIBI: ", msg_recv_robot)
                    break_loop = gripper_action(args.communication_mode, msg_recv_robot, sock_comm_glove)
        if break_loop:
            break
    pub_sub_gripper.unregister()
