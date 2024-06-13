#!/usr/bin/env python3

class TrajectoryLimitator():

    def __init__(self):
        self.min_limits = [-0.400, 0.740, 0.600] 
        self.max_limits = [0.400, 0.500, 0.200] #ADELANTE

    def get_min_limit(self, coord):
        return min(self.min_limits[coord], self.max_limits[coord])
    
    def get_max_limit(self, coord):
        return max(self.min_limits[coord], self.max_limits[coord])

    def limit_coordinate(self, pose_element, coord):
        if pose_element[coord] < self.get_min_limit(coord):
            pose_element[coord] = self.get_min_limit(coord)
        if pose_element[coord] > self.get_max_limit(coord):
            pose_element[coord] = self.get_max_limit(coord)
        return pose_element[coord]

    def limit_movement(self, pose_element):
        for i in range(len(["x", "y", "z"])):
            pose_element[i] = self.limit_coordinate(pose_element, i)
        return pose_element