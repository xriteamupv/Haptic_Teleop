#!/usr/bin/env python3
class HumanArm:

    def __init__(self):
        self.min_limits = [0, 0, -0.02]
        self.max_limits = [1, 1, -0.05]
        self.pose_list = [0.0, 0.0, 0.0]