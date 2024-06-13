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
pub_topic = rospy.Publisher('teleop_robot_gripper', String, queue_size=1)

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument( "--max_width", type = float, default = 100.0 )
    parser.add_argument( "--max_force", type = float, default = 40.0 )
    parser.add_argument( "--start_width", type = float, default = 100.0 )
    parser.add_argument( "--width_tolerance", type = float, default = 2.0 )
    parser.add_argument( "--force_tolerance", type = float, default = 2.0 )
    parser.add_argument( "--max_duration", type = float, default = 60.0 ) # mseconds
    parser.add_argument( "--time_max_width", type = float, default = 2.0 ) # seconds
    parser.add_argument( "--static_force", type = float, default = 39.0 ) # Netwons
    parser.add_argument( "--width_model", type = int, default = 0 )
    parser.add_argument( "--force_model", type = int, default = 0 )
    parser.add_argument( "--delay_model", type = int, default = 1 )
    parser.add_argument( "--grip_levels", type = int, default = 4 ) # CHECK 3 or 4
    parser.add_argument( "--bidirectional", type = bool, default = False ) #PENDING
    args = parser.parse_args()
    return args

def analyze_inputs(args):
    c = 1 # PENDING

def get_position_data(str_received):
    str_pos = str_received.split("[")[1]
    str_pos = str_pos.split("]")[0]
    str_pos = str_pos.split(", ")
    return str_pos

def update_position(data):
    rospy.loginfo(rospy.get_caller_id() + " SYSTEM: I heard %s", data.data)
    msg_received = get_position_data(data.data)
    robot_control = [round(float(msg),3) for msg in msg_received[0:3]]
    gripper_manager.update_time_received(msg_received)
    gripper_manager.process_grip_level(int(msg_received[4])) 
    gripper_manager.configure_force(round(float(msg_received[5]),3))
    if gripper_manager.is_movement_detected():
        print("RECIBI ALGO DIFERENTE A 0.0")
        if robot_control == [1.0, 1.0, 1.0]:
            gripper_manager.on_open_or_close()
        else:
            gripper_manager.gripper_operation()
            #if gripper_manager.objective_changed:
            #    time_delta = gripper_manager.time_received - gripper_manager.current_time
            #    time_diff_sec = round(float(time_delta.microseconds)*0.000001, 3) 
            #    #print("TIME_DIFF: ", time_diff_sec)
            #    time.sleep(3)
            if args.bidirectional:
                lst_to_send = []
                lst_to_send.extend(msg_received[4:6])
                lst_to_send.append(0) #gripper_manager.result)
                msg_position = str(lst_to_send)
                rospy.loginfo(msg_position)
                pub_topic.publish(msg_position)
    else:
        print("NO MOVEMENT DETECTED")

def update_gripper_state(data):
    #rospy.loginfo(rospy.get_caller_id() + " GRIPPER: I heard %s", data.data)
    rec_width = round(float(data.data), 3)
    if gripper_manager.is_width_correct(rec_width):
        gripper_manager.update_width(rec_width)
        gripper_manager.update_detection()

if __name__ == '__main__':
    print("MAIN")
    do_update = True
    args = get_args()
    analyze_inputs(args)
    sock_comm_track = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_comm_track.bind(('192.168.1.102', 23456))
    sock_comm_track.listen(5)
    comm, addr = sock_comm_track.accept()
    print("Got connection from", addr)
    print("LINK ESTABLISHED. Waiting for commands from Tracking Program ...")
    sock_comm_glove = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_comm_glove.connect(('10.45.26.2', 34567))
    rospy.init_node('gripper_control', anonymous=True)
    gripper_manager = GripClient(args)
    pub_sub = rospy.Subscriber("/gripper_controller/get_pos", Float32, callback=update_gripper_state, queue_size=1)
    while True:
        message = comm.recv(55)
        msg_recv = message.decode()
        if len(msg_recv) > 1:
            if msg_recv[0]=="[" and msg_recv[len(msg_recv)-1]=="]":
                print("RECIBI: ", msg_recv)
                msg_received = get_position_data(msg_recv)
                gripper_control = [int(msg) for msg in msg_received[0:2]]
                gripper_manager.update_time_received(msg_received)
                gripper_manager.process_grip_level(int(msg_received[0])) 
                gripper_manager.configure_force(round(args.static_force,3))
                if gripper_control == [1, 1]:
                        print("CERRANDO...")
                        gripper_manager.on_open_or_close()
                        msg_send = [1.0, 1.0, 0]
                        msg_send.append(gripper_manager.event_manager.time_received.strftime("%H:%M:%S:%f"))
                        msg_send_glove = str(msg_send)
                        sock_comm_glove.send(msg_send_glove.encode())
                        break
                elif gripper_manager.is_movement_detected():
                    gripper_manager.gripper_operation()
                    msg_send = []
                    msg_send.append(gripper_manager.get_width_variation())
                    msg_send.append(gripper_manager.get_grip_force())
                    msg_send.append(int(msg_received[5])) # FINGERS
                    msg_send.append(gripper_manager.event_manager.time_received.strftime("%H:%M:%S:%f"))
                    msg_send_glove = str(msg_send)
                    sock_comm_glove.send(msg_send_glove.encode())
                    if args.bidirectional:
                        lst_to_send = []
                        lst_to_send.extend(msg_received[4:6])
                        lst_to_send.append(0) #gripper_manager.result)
                        msg_position = str(lst_to_send)
                        #rospy.loginfo(msg_position)
                        #pub_topic.publish(msg_position)
                else:
                    print("NO MOVEMENT DETECTED")
    pub_sub.unregister()
