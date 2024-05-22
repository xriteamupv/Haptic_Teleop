#!/usr/bin/env python3
import argparse
from datetime import datetime
import cv2 as cv
from utils import CvFpsCalc
import rospy
from std_msgs.msg import String
from utils.HandTracker import HandTracker
from utils.Model import Model

#global confirmation_received 

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)
    parser.add_argument('--use_static_image_mode', action='store_true')
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence', type=float, default=0.7)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence', type=int, default=0.5)
    parser.add_argument("--initial", type=int, default=1) # [0, 1, 2] = [lower, centered, upper] PENDING
    parser.add_argument("--tracking", type=int, default=0) # CHECKED
    parser.add_argument("--record", type=str, default="") # CKECKED
    parser.add_argument("--withangle", type=bool, default=False) # PENDING
    parser.add_argument("--bidirectional", type=bool, default=False) # CHECKED

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

def track_human_arm():
    args = get_args()
    pub_topic = rospy.Publisher('teleop_arm_robot', String, queue_size=1)
    if args.bidirectional:
        rospy.Subscriber("teleop_robot_arm", String, callback=receive_confirmation, queue_size=1)
    rospy.init_node('arm_tracking', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    global confirmation_received
    confirmation_received = True
    #print("BIDIRECTIONAL: ", args.bidirectional)

    hand_tracker = HandTracker(args)
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    frame_width = int(hand_tracker.content.cap.get(3)) 
    frame_height = int(hand_tracker.content.cap.get(4)) 
    second_time = datetime.now()
    video_out = cv.VideoWriter(str(args.record)+".mp4", cv.VideoWriter_fourcc(*"mp4v"), 20, (frame_width, frame_height)) 
    
    while True: 

        key = cv.waitKey(10)
        if key == 27:  # ESC
            break

        hand_tracker.capture_image()
        if not hand_tracker.get_content_ret():
            break
        Model.select_mode(key)

        # DETECTING THE BLUE HAPTIC GLOVE
        #hand_tracker.image = cv.cvtColor(hand_tracker.image, cv.COLOR_BGR2RGB)

        hand_tracker.read_content_fps(cvFpsCalc.get())
        hand_tracker.main_process()
        Model.logging_csv(hand_tracker.get_pre_processed_landmark_list(), hand_tracker.get_pre_processed_point_history_list())
        Model.draw_mode_info(hand_tracker.get_display_image())
        
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
                msg_position = str(lst_to_send) #str(hand_tracker.mean_landmarks_coords)
                rospy.loginfo(msg_position)
                pub_topic.publish(msg_position)
                confirmation_received = False
                #rate.sleep()

    hand_tracker.content.cap.release()
    cv.destroyAllWindows()
    datetime_str = second_time.strftime("%H:%M:%S:%f")
    msg_close = str([1.0,1.0,1.0,datetime_str])
    rospy.loginfo(msg_close)
    pub_topic.publish(msg_close)
    print("CERRANDO CONEXION...")
        
if __name__ == '__main__':
    try:
        track_human_arm()
    except rospy.ROSInterruptException:
        pass