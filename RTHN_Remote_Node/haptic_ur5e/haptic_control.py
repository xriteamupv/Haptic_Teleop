import sys
import socket
import argparse
from utils.HapticClient import HapticClient
from utils.Exceptions import *
from datetime import datetime

global args
global haptic_manager

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument( "--max_fingers", type = int, default = 6 )
    parser.add_argument( "--min_intensity", type = float, default = 10.0 )
    parser.add_argument( "--max_intensity", type = float, default = 100.0 )
    parser.add_argument( "--precision_duration", type = float, default = 100.0 )
    parser.add_argument( "--precision_width", type = float, default = 3.3 )
    parser.add_argument( "--initial_delay", type = float, default = 0.0 )
    parser.add_argument( "--final_delay", type = float, default = 0.0 )
    parser.add_argument( "--width_weight", type = float, default = 0.7 )
    parser.add_argument( "--width_threshold", type = float, default = 70 )
    parser.add_argument( "--intensity_shift", type = float, default = 0.0 )
    parser.add_argument( "--static_intensity", type = float, default = 80.0 )
    parser.add_argument( "--intensity_model", type = int, default = 0 )
    parser.add_argument( "--max_width", type = float, default = 100.0 )
    parser.add_argument( "--max_force", type = float, default = 40.0 )
    parser.add_argument( "--right_hand_enabled", type = bool, default = True )
    parser.add_argument( "--discrete_sensations", type = bool, default = True )
    parser.add_argument( "--bidirectional_comms", type = bool, default = False )
    args = parser.parse_args()
    return args

def analyze_inputs(args):
    c = 1 # PENDING

def get_haptic_data(str_received):
    str_pos = str_received.split("[")[1]
    str_pos = str_pos.split("]")[0]
    str_pos = str_pos.split(", ")
    return str_pos

if __name__ == '__main__':
    print("MAIN")
    args = get_args()
    analyze_inputs(args)
    haptic_manager = HapticClient(args)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('192.168.1.220', 34567))
    s.listen(5)
    c, addr = s.accept()
    print('Got connection from', addr)
    print("LINK ESTABLISHED. Waiting for commands from Local Node...")
    while True:
        message = c.recv(1024)
        msg_recv = message.decode()
        if msg_recv:
            if msg_recv[0]=="[" and msg_recv[len(msg_recv)-1]=="]":
                msg_received = get_haptic_data(msg_recv)
                print("RECIBI: ", msg_received)
                haptic_manager.gripper.grip_width_var = round(float(msg_received[0]),3)
                haptic_manager.gripper.grip_force = float(msg_received[1])
                haptic_manager.configure_fingers(int(msg_received[2]))
                if haptic_manager.is_close_detected():
                        print("CERRANDO...")
                        haptic_manager.on_close()
                        break
                elif haptic_manager.is_haptic_grip_detected():
                    print("HAPTIC DETECTED")
                    haptic_manager.time_received = datetime.strptime(str(msg_received[3][1:(len(msg_received[3])-1)]), "%H:%M:%S:%f")
                    haptic_manager.activate_glove()
                    if args.bidirectional_comms:
                        lst_to_send = []
                        lst_to_send.extend(haptic_manager.get_return_message())
                        lst_to_send.append(msg_received[3]) #Can be changed for LATENCY STUDY
                        lst_to_send.append(0) #haptic_manager.result)
                        msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                        #rospy.loginfo(msg_position)
                        #pub_topic.publish(msg_position)
                else:
                    haptic_manager.turn_gloves_off()
                haptic_manager.update_prev_width()
            else:
                print("RECIBI NADA")