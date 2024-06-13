#!/usr/bin/env python3
import rospy
import argparse
import socket
from datetime import datetime
import cv2 as cv
from std_msgs.msg import String
from utils.HandTracker import HandTracker
from utils.GestureModel import GestureModel
from utils.Exceptions import *

#global confirmation_received 

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument( "--device", type = int, default = 0 )
    parser.add_argument( "--width", type = int, default = 960 )
    parser.add_argument( "--height", type = int, default = 540 )
    parser.add_argument( '--use_static_image_mode', action = 'store_true')
    parser.add_argument( "--min_detection_confidence", type = float, default = 0.7 )
    parser.add_argument( "--min_tracking_confidence", type = float, default = 0.5 )
    parser.add_argument( "--initial_position", type = int, default = 4 ) # [0, 1, ..., 8] = [up_left, up_center, up_right, ..., low_right]
    parser.add_argument( "--tolerance", type = int, default = 1 ) # [0, 1, 2] = [low, medium, high]
    parser.add_argument( "--tracking", type = int, default = 0 )
    parser.add_argument( "--gloves_color", type = int, default = 0 ) # [0, 1, 2, 3] = [no gloves, blue, yellow, black] PENDING
    parser.add_argument( "--record", type = str, default = "")
    parser.add_argument( "--with_orientation", type = bool, default = False) # PENDING
    parser.add_argument( "--bidirectional", type = bool, default = False)

    args = parser.parse_args()

    return args

def get_position_data(str_received):
    str_pos = str_received.split("[")[1]
    str_pos = str_pos.split("]")[0]
    str_pos = str_pos.split(", ")
    return str_pos

def receive_confirmation(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    msg_received = get_position_data(data.data)
    #if int(msg_received[4]) == 0 # Successful movement
    print("ENTRO A CALLBACK")
    global confirmation_received
    confirmation_received = True

def analyze_inputs(args):
    if args.width < 360 or args.width > 1080:
        raise IncorrectWidth(args.width)
    if args.height < 240 or args.height > 1080:
        raise IncorrectHeight(args.height)
    if args.min_detection_confidence < 0.0 or args.min_detection_confidence > 1.0:
        raise IncorrectDetectionConfidence(args.min_detection_confidence)
    if args.min_tracking_confidence < 0.0 or args.min_tracking_confidence > 1.0:
        raise IncorrectTrackingConfidence(args.min_tracking_confidence)
    if args.initial_position < 0 or args.initial_position > 8:
        raise IncorrectInitialPosition(args.initial_position)
    if args.tolerance < 0 or args.tolerance > 2:
        raise IncorrectTolerance(args.tolerance)
    if args.tracking < 0 or args.tracking > 2:
        raise IncorrectTracking(args.tracking)
    if args.gloves_color < 0 or args.gloves_color > 3:
        raise IncorrectGlovesColor(args.gloves_color)

def track_human_arm(args, sock_comm):
    #pub_topic = rospy.Publisher('teleop_arm_robot', String, queue_size=1)
    #if args.bidirectional:
    #    rospy.Subscriber("teleop_robot_arm", String, callback=receive_confirmation, queue_size=1)
    #rospy.init_node('arm_tracking', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    global confirmation_received
    confirmation_received = True
    
    hand_tracker = HandTracker(args)
    second_time = datetime.now()
    if args.record != "":
        frame_width = int(hand_tracker.content.cap.get(3)) 
        frame_height = int(hand_tracker.content.cap.get(4)) 
        video_out = cv.VideoWriter(str(args.record)+".mp4", cv.VideoWriter_fourcc(*"mp4v"), 20, (frame_width, frame_height)) 
    
    while True: 

        key = cv.waitKey(10)
        if key == 27:  # ESC
            break

        hand_tracker.capture_image()
        if not hand_tracker.get_content_ret():
            break
        GestureModel.select_mode(key)

        # DETECTING THE BLUE HAPTIC GLOVE
        #hand_tracker.image = cv.cvtColor(hand_tracker.image, cv.COLOR_BGR2RGB)
        hand_tracker.configure_detection(args.gloves_color)

        hand_tracker.read_content_fps(hand_tracker.get_fps())
        hand_tracker.main_process()
        GestureModel.logging_csv(hand_tracker.get_pre_processed_landmark_list(), hand_tracker.get_pre_processed_point_history_list())
        GestureModel.draw_mode_info(hand_tracker.get_display_image())
        
        hand_tracker.draw_info()
        if args.record != "":
            video_out.write(hand_tracker.get_display_image())
        cv.imshow('Hand Gesture Recognition', hand_tracker.get_display_image())
        
        if hand_tracker.state.robot_state == "Running":
            #if not confirmation_received:
            enable_bidirectional = args.bidirectional and confirmation_received
            print("CONFIRMATION RECEIVED: ", confirmation_received)
            if enable_bidirectional or not args.bidirectional:
                print("ENTRO IF")
                second_time = datetime.now()
                datetime_str = second_time.strftime("%H:%M:%S:%f")
                lst_to_send = []
                lst_to_send.extend(hand_tracker.get_mean_landmarks_coords())
                lst_to_send.append(datetime_str) #time_delta.microseconds / 0.001)
                if hand_tracker.grip_detected:
                    lst_to_send.append(hand_tracker.grip_level) # level => width close
                else:
                    lst_to_send.append(0) # level => width open
                lst_to_send.append(30.0) # force
                msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                #rospy.loginfo(msg_position)
                #pub_topic.publish(msg_position)
                sock_comm.send(msg_position.encode())
                confirmation_received = False
                #rate.sleep()

    hand_tracker.content.cap.release()
    cv.destroyAllWindows()
    datetime_str = second_time.strftime("%H:%M:%S:%f")
    msg_close = str([1.0,1.0,1.0,datetime_str,0,30.0])
    sock_comm.send(msg_close.encode())
    #rospy.loginfo(msg_close)
    #pub_topic.publish(msg_close)
    print("CERRANDO CONEXION...")
        
if __name__ == '__main__':
    args = get_args()
    analyze_inputs(args)
    sock_comm = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_comm.connect(('10.45.26.9', 12345))
    track_human_arm(args, sock_comm)