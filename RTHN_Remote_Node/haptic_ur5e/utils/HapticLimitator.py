#!/usr/bin/env python3

class HapticLimitator():

    def __init__(self, args):
        self.fingers_limits = [0, args.max_fingers]
        self.intensity_limits = [args.min_intensity, args.max_intensity]
        self.sensation_delays = [args.initial_delay, args.final_delay]
        self.grip_width_limits = [0.0, args.max_width] # millimeters
        self.grip_force_limits = [0.0, args.max_force] # Newtons
        self.weights = [args.width_weight, 1-args.width_weight] # width_weight, force_weight
        self.thresholds = [args.width_threshold, 100-args.width_threshold] # width_threshold, force_threshold

    def configure_fingers(self, fingers_recv):
        current_fingers = fingers_recv
        if fingers_recv >= self.fingers_limits[1]:
            current_fingers = self.fingers_limits[1]
        elif fingers_recv <= self.fingers_limits[0]:
            current_fingers = self.fingers_limits[0]
        return current_fingers
            