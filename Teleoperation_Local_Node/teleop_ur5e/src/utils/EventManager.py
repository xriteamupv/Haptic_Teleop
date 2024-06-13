#!/usr/bin/env python3
from datetime import datetime


class EventManager():

    def __init__(self):
        self.grip_detected = False
        self.object_detected = False
        self.objective_changed = False
        self.time_received = datetime.now()
        self.current_time = datetime.now()

    def not_objective_changed(self):
        self.objective_changed = False

    def yes_objective_changed(self):
        self.objective_changed = True

    def detect_grip(self, grip_level):
        self.grip_detected = True
        if grip_level == 0:
            self.grip_detected = False

    def get_time_difference(self):
        return self.time_received - self.current_time
    
    def update_current_time(self):
        self.current_time = self.time_received

    def update_detection(self, gripper):
        self.object_detected = gripper.get_width_variation() >= 10 and gripper.objective_width < gripper.current_width

    def update_time_received(self, msg_received):
        self.time_received = datetime.strptime(str(msg_received[2][1:(len(msg_received[2])-1)]), "%H:%M:%S:%f")