#!/usr/bin/env python3
from time import sleep
from datetime import datetime
from utils.HapticMapper import HapticMapper

class HapticGlove():

    def __init__(self, args):
        self.precision_duration = args.precision_duration
        self.intensity_shift = args.intensity_shift
        self.intensity_model = args.intensity_model
        self.static_intensity = args.static_intensity
        self.right_hand_enabled = args.right_hand_enabled
        self.discrete_sensations = args.discrete_sensations
        self.current_intensity = args.max_intensity
        self.current_fingers = args.max_fingers

    def are_fingers_detected(self):
        fingers_received = self.current_fingers >= 2
        return fingers_received
    
    def is_hand_closed(self):
        closed_grip_received = self.current_fingers == 6
        return closed_grip_received
    
    def configure_actuators(self):
        haptic_actuators = []
        if self.current_fingers >= 1:
            haptic_actuators.append({"index": 0, "intensity": int(self.current_intensity)})
            print("ACTIVO 1")
        if self.current_fingers >= 2:
            haptic_actuators.append({"index": 1, "intensity": int(self.current_intensity)})
            print("ACTIVO 2")
        if self.current_fingers >= 3:
            haptic_actuators.append({"index": 2, "intensity": int(self.current_intensity)})
            print("ACTIVO 3")
        if self.current_fingers >= 4:
            haptic_actuators.append({"index": 3, "intensity": int(self.current_intensity)})
            print("ACTIVO 4")
        if self.current_fingers >= 5:
            haptic_actuators.append({"index": 4, "intensity": int(self.current_intensity)})
            print("ACTIVO 5")
        if self.current_fingers == 6:
            haptic_actuators.append({"index": 5, "intensity": int(self.current_intensity)})
            print("ACTIVO 6")
        print("ACT: ", haptic_actuators)
        return haptic_actuators
    
    def configure_current_intensity(self, grip_width_var, grip_force, limitator):
        if self.intensity_model == 1: # LINEAR INTERPOLATION
            self.current_intensity = HapticMapper.linear_interpolation(grip_width_var, grip_force, limitator)
        elif self.intensity_model == 2: # WEIGHTED INTERPOLATION
            self.current_intensity = HapticMapper.weighted_interpolation(grip_width_var, grip_force, limitator)
        elif self.intensity_model == 3: # DYNAMIC WEIGHTED INTERPOLATION
            self.current_intensity = HapticMapper.dynamically_weighted_interpolation(grip_width_var, grip_force, limitator)
        elif self.intensity_model == 4:
            self.current_intensity = HapticMapper.normalized_biased_weighted_interpolation(grip_width_var, grip_force, limitator)
        else: # STATIC INTENSITY
            self.current_intensity = self.static_intensity

    def on_close(self):
        self.current_intensity = 0.0
        self.current_fingers = 0.0