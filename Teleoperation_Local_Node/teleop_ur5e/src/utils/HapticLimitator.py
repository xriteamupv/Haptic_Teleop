#!/usr/bin/env python3

class HapticLimitator():

    def __init__(self, args):
        self.fingers_limits = [0, args.max_fingers]
        self.intensity_limits = [args.min_intensity, args.max_intensity]
        self.sensation_delays = [args.initial_delay, args.final_delay]
        self.grip_width_limits = [0.0, args.max_width] # millimeters
        self.grip_force_limits = [0.0, args.max_force] # Newtons