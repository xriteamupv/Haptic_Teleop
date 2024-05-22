#!/usr/bin/env python3

class State:

    def __init__(self):
        self.robot_state = "Stopped"
        self.count_state = 20

    def change_robot_state(self):
        if self.robot_state == "Stopped":
            self.robot_state = "Running"
        else:
            self.robot_state = "Stopped"

    def increment_count(self):
        self.count_state = self.count_state + 1

    def initialize_count(self):
        self.count_state = 0

    def is_sufficient_count(self):
        return self.count_state > 50