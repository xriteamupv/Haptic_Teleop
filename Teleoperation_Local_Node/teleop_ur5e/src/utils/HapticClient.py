#!/usr/bin/env python3
from time import sleep
from datetime import datetime
from utils.HapticLimitator import HapticLimitator
from utils.HapticMapper import HapticMapper
from bhaptics import better_haptic_player as player
from bhaptics.better_haptic_player import BhapticsPosition

class HapticClient():

    def __init__(self, args):
        self.limitator = HapticLimitator(args)
        self.precision_duration = args.precision_duration
        self.right_hand_enabled = args.right_hand_enabled
        self.discrete_sensations = args.discrete_sensations
        self.intensity_shift = args.intensity_shift
        self.intensity_model = args.intensity_model
        self.static_intensity = args.static_intensity
        self.weights = [0.7, 0.3] # width_weight, force_weight
        self.thresholds = [70, 30] # width_threshold, force_threshold
        self.grip_force = 40.0
        self.grip_width_var = 0.0
        self.grip_close_detected = False
        self.time_received = datetime.now()
        self.current_time = datetime.now()
        
        player.initialize()
        print("register CenterX")
        player.register("CenterX", "CenterX.tact")
        print("register Circle")
        player.register("Circle", "Circle.tact")
        self.sample_initial_sensation()

    def sample_initial_sensation(self):
        self.current_intensity = self.limitator.intensity_limits[1]
        self.current_fingers = self.limitator.fingers_limits[1]
        self.activate_fingers(self.precision_duration)

    def update_current_time(self):
        self.current_time = self.time_received

    def configure_fingers(self, fingers_recv):
        if fingers_recv >= self.limitator.fingers_limits[1]:
            self.current_fingers = self.limitator.fingers_limits[1]
        elif fingers_recv <= self.limitator.fingers_limits[0]:
            self.current_fingers = self.limitator.fingers_limits[0]
        else:
            self.current_fingers = fingers_recv

    def get_return_message(self):
        return [self.grip_width_var, self.grip_force, self.current_fingers]
    
    def is_haptic_grip_detected(self):
        width_var_received = self.grip_width_var != 0.0
        force_received = self.grip_force != 0.0
        fingers_received = self.current_fingers != 0
        return width_var_received and force_received and fingers_received
    
    def configure_actuators(self):
        haptic_actuators = []
        if self.current_fingers >= 1:
            haptic_actuators.append({"index": 0, "intensity": self.current_intensity})
        if self.current_fingers >= 2:
            haptic_actuators.append({"index": 1, "intensity": self.current_intensity})
        if self.current_fingers >= 3:
            haptic_actuators.append({"index": 2, "intensity": self.current_intensity})
        if self.current_fingers >= 4:
            haptic_actuators.append({"index": 3, "intensity": self.current_intensity})
        if self.current_fingers == 5:
            haptic_actuators.append({"index": 4, "intensity": self.current_intensity})
        
        if self.grip_close_detected:
            haptic_actuators.append({"index": 5, "intensity": self.current_intensity})

        return haptic_actuators

    def activate_fingers(self, duration_msec):
        glove_actuators = self.configure_actuators()
        if self.right_hand_enabled:
            player.submit_dot("gloveRFrame", BhapticsPosition.GloveR.value, glove_actuators, duration_msec)
        else:
            player.submit_dot("gloveLFrame", BhapticsPosition.GloveL.value, glove_actuators, duration_msec)
        sleep(duration_msec*0.001) # Duration in seconds needed
    
    def configure_current_intensity(self):
        if self.intensity_model == 1: # LINEAR INTERPOLATION
            self.current_intensity = HapticMapper.linear_interpolation(self.grip_width_var, self.grip_force, self.limitator)
        elif self.intensity_model == 2: # WEIGHTED INTERPOLATION
            self.current_intensity = HapticMapper.weighted_interpolation(self.grip_width_var, self.grip_force, self.limitator, self.weights)
        elif self.intensity_model == 3: # DYNAMIC WEIGHTED INTERPOLATION
            self.current_intensity = HapticMapper.dynamically_weighted_interpolation(self.grip_width_var, self.grip_force, self.limitator, self.thresholds)
        elif self.intensity_model == 4:
            self.current_intensity = HapticMapper.normalized_biased_weighted_interpolation(self.grip_width_var, self.grip_force, self.limitator)
        else: # STATIC INTENSITY
            self.current_intensity = self.static_intensity

    def activate_glove(self):
        time_delta = self.time_received - self.current_time
        time_diff_msec = round(float(time_delta.microseconds)*0.001, 3) 
        # MAPEAR FORCE Y WIDTH VAR TO INTENSITY
        self.configure_current_intensity()
        # CONSIDERAR TODOS LOS DELAYS, Y LIMITACIONES INTENSITY
        if self.discrete_sensations:
            # LOOPING THROUGH DISCRETIZED PRECISION_DURATION TIMES
            time_msec = 0.0
            while time_msec <= time_diff_msec:
                self.activate_fingers(self.precision_duration)
                time_msec = time_msec + self.precision_duration
        else:
            # USING CURRENT_TIME - TIME_RECEIVED CONTINOUS TIME
            self.activate_fingers(time_diff_msec)
        self.update_current_time()

    def is_close_detected(self):
        width_var_on_close = self.grip_width_var == 1.0
        force_on_close = self.grip_force == 1.0
        return width_var_on_close and force_on_close
    
    def on_close(self):
        self.current_intensity = 0.0
        self.current_fingers = 0.0
        self.activate_fingers(self.precision_duration)
        player.destroy()