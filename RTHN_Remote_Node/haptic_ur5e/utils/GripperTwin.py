#!/usr/bin/env python3
from time import sleep
from datetime import datetime

class GripperTwin():

    def __init__(self, args):
        self.grip_force = 40.0
        self.prev_width_var = 0.0
        self.grip_width_var = 0.0
        self.grip_close_detected = False
        self.object_detected = False
        self.precision_width = args.precision_width
        
    def is_grip_detected(self):
        prev_width_var_received = self.prev_width_var >= self.precision_width
        width_var_received = self.grip_width_var >= self.precision_width
        seq_width_variation = abs(self.grip_width_var - self.prev_width_var) <= self.precision_width
        force_received = self.grip_force != 0.0
        print("GRIP INFO: ", [self.prev_width_var, self.grip_width_var, self.grip_force])
        print("GRIP DETECTED: ", [prev_width_var_received, width_var_received, seq_width_variation, force_received])
        return prev_width_var_received and width_var_received and seq_width_variation and force_received
    
    def is_close_detected(self):
        width_var_on_close = self.grip_width_var == 1.0
        force_on_close = self.grip_force == 1.0
        return width_var_on_close and force_on_close
    
    def update_prev_width(self):
        self.prev_width_var = self.grip_width_var