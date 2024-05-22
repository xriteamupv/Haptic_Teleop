#!/usr/bin/env python3
from utils.Tracking import Tracking
from utils.Visualization import Visualization
from utils.State import State
from utils.History import History

class HandTracker:

    def __init__(self, args):
        self.content = Visualization(args)
        self.tracker = Tracking(args)
        self.state = State()
        self.history = History()
        self.type = args.tracking

    def get_processing_image(self):
        return self.content.image

    def get_display_image(self):
        return self.content.debug_image
    
    def get_display_image_properties(self):
        return self.get_display_image().shape[1], self.get_display_image().shape[0]
    
    def get_content_ret(self):
        return self.content.ret
    
    def get_mean_landmarks_coords(self):
        return self.tracker.mean_landmarks_coords
    
    def get_mean_landmarks_list(self):
        return self.tracker.mean_landmarks_list
    
    def get_tracker_results(self):
        return self.tracker.results
    
    def get_keypoint_classifier(self, landmark_list):
        return self.tracker.keypoint_classifier(landmark_list)
    
    def get_point_history_classifier(self):
        return self.tracker.point_history_classifier(self.history.pre_processed_point_history_list)
    
    def get_pre_processed_landmark_list(self):
        return self.history.pre_processed_landmark_list
    
    def get_pre_processed_point_history_list(self):
        return self.history.pre_processed_point_history_list
    
    def get_hand_sign_id(self):
        return self.tracker.hand_sign_id
    
    def get_keypoint_labels(self):
        return self.tracker.get_keypoint_labels()
    
    def get_robot_state(self):
        return self.state.robot_state
    
    def read_content_fps(self, new_fps):
        self.content.fps = new_fps
    
    def draw_bounding_rect(self):
        self.content.draw_bounding_rect(self.tracker.brect)

    def draw_landmarks_keypoints(self, landmark_list):
        self.content.draw_landmarks(landmark_list, self.type)
        self.content.draw_keypoints(landmark_list, self.type)
    
    def draw_info_text(self, handedness):
        self.content.draw_info_text(self.tracker.brect,
                    handedness,
                    self.get_keypoint_labels(),
                    self.get_robot_state(),
                    self.get_mean_landmarks_coords()
                )
        
    def draw_point_history(self, color):
        self.content.draw_point_history(self.history.point_history, color)

    def draw_info(self):
        self.content.draw_info()

    def process_hand_detection(self):
        self.content.set_writeable(False)
        self.tracker.process_tracking(self.get_processing_image())
        self.content.set_writeable(True)

    def capture_image(self):
        self.content.capture_image()
        self.process_hand_detection()

    def configure_hand_sign_id(self):
        return self.tracker.configure_hand_sign_id(self.history.pre_processed_landmark_list)

    def is_hand_near_reference(self):
        return self.tracker.is_hand_near_reference()
    
    def calc_bounding_rect(self, landmarks):
        image_width, image_height = self.get_display_image_properties()
        self.tracker.get_bounding_rect(landmarks, [image_width, image_height])
    
    def calc_landmark_list(self, landmarks):
        image_width, image_height = self.get_display_image_properties()
        return self.tracker.get_landmark_list(landmarks, [image_width, image_height])

    def pre_process_landmark(self, landmark_list):
        return self.tracker.pre_process_landmark(landmark_list)

    def pre_process_point_history(self):
        image_width, image_height = self.get_display_image_properties()
        return self.tracker.pre_process_point_history(self.history.point_history, [image_width, image_height])
    
    def set_new_mean_landmarks_coords(self, coords_xyz, len_coord):
        self.tracker.set_new_mean_landmarks_coords(coords_xyz, len_coord)

    def get_coords_hand_type(self, landmark_coords, index):
        coords = []
        for i in range(len(landmark_coords)):
            bool_little_finger = i in [18,19,20]
            bool_anular_finger = i in [14,15,16]
            bool_middle_finger = i in [10,11,12]
            bool_index_finger = i in [6,7,8]
            bool_thumb_finger = i in [3,4]
            bool_palm = i in [0,1,2,5,9,13,17]
            if self.type in [0,1] and (bool_thumb_finger or bool_index_finger) :
                coords.append(landmark_coords[i][index])
            if self.type == 0 and (bool_middle_finger or bool_anular_finger or bool_little_finger):
                coords.append(landmark_coords[i][index])
            if bool_palm:
                coords.append(landmark_coords[i][index])
        return coords

    def configure_mean_landmarks_coord(self, landmark_coords):
        coords_x = self.get_coords_hand_type(landmark_coords, 0) #[coord[0] for coord in landmark_coords]
        coords_y = self.get_coords_hand_type(landmark_coords, 1) #[coord[1] for coord in landmark_coords]
        coords_z = self.get_coords_hand_type(landmark_coords, 2) #[coord[2] for coord in landmark_coords]
        len_coord = len(coords_x)
        #print("LEN COORD: ", len(coords_x))
        #print("LEN COORD 2: ", len(landmark_coord[0]))
        if(len_coord == 0):
            len_coord = 0.000001
        self.set_new_mean_landmarks_coords([coords_x, coords_y, coords_z], len_coord)

    def configure_mean_landmarks_list(self, landmark_list):
        landmarks_x = self.get_coords_hand_type(landmark_list, 0) #[landmark[0] for landmark in landmark_list]
        landmarks_y = self.get_coords_hand_type(landmark_list, 1) #[landmark[1] for landmark in landmark_list]
        #all_land_z = [coord[2] for coord in landmark_list]
        len_land = len(landmarks_x)
        if(len_land == 0):
            len_land = 0.000001
        #self.get_mean_landmarks_coords() = [0, 0]
        self.get_mean_landmarks_list()[0] = int(sum(landmarks_x)/len_land)
        self.get_mean_landmarks_list()[1] = int(sum(landmarks_y)/len_land)

    def calculate_landmark(self, hand_landmarks):
        landmark_list, landmark_coord = self.calc_landmark_list(hand_landmarks)
        self.configure_mean_landmarks_coord(landmark_coord)
        self.configure_mean_landmarks_list(landmark_list)
        # Conversion to relative coordinates / normalized coordinates
        self.history.pre_processed_landmark_list = self.pre_process_landmark(landmark_list)
        self.history.pre_processed_point_history_list = self.pre_process_point_history()
        return landmark_list
    
    def hand_sign_classification(self):
        bool_near_reference = self.is_hand_near_reference()
        self.configure_hand_sign_id()
        self.history.append_point_history(self.get_mean_landmarks_list())
        self.state.increment_count()
        if self.get_hand_sign_id() == 3 and bool_near_reference and self.state.is_sufficient_count():
            self.state.change_robot_state()
            self.state.initialize_count()

    def main_process(self):
        #bool_near_reference = self.is_hand_near_reference()

        if self.get_tracker_results().multi_hand_landmarks is not None:
            
            for hand_landmarks, handedness in zip(self.get_tracker_results().multi_hand_landmarks,
                                                  self.get_tracker_results().multi_handedness):
                
                self.calc_bounding_rect(hand_landmarks)
                landmark_list = self.calculate_landmark(hand_landmarks)
                self.hand_sign_classification()

                # Finger gesture classification
                finger_gesture_id = 0
                point_history_len = len(self.history.pre_processed_point_history_list)
                if point_history_len == (self.history.history_length * 2):
                    finger_gesture_id = self.get_point_history_classifier()

                # Calculates the gesture IDs in the latest detection (USEFUL?)
                self.history.append_gesture(finger_gesture_id)
                most_common_fg_id = self.history.find_most_common_gesture()

                # Drawing part
                self.draw_bounding_rect()
                self.draw_landmarks_keypoints(landmark_list)
                self.draw_info_text(handedness)
        else:
            self.history.empty_point_history()

        if(self.is_hand_near_reference()):
            self.draw_point_history((152, 251, 152))
        else:
            self.draw_point_history((0, 0, 255))
    
    
