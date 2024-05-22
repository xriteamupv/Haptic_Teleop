#!/usr/bin/env python3
class Gripper:

    def __init__(self):
        self.width_limits = [0.0, 100.0] # millimeters
        self.force_limits = [0.0, 120.0] # Newtons
        self.grip_detected = False
        self.current_width = 100.0 # open
        self.current_force = 0.0