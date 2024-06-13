#!/usr/bin/env python3

class GripLimitator():

    def __init__(self, args):
        self.width_limits = [0.0, args.max_width] # millimeters
        self.force_limits = [0.0, args.max_force] # Newtons

    def get_min_width(self):
        return self.width_limits[0]
    
    def get_max_width(self):
        return self.width_limits[1]
    
    def get_min_force(self):
        return self.force_limits[0]
    
    def get_max_force(self):
        return self.force_limits[1]
    
    def get_width_range(self):
        return self.width_limits[1] - self.width_limits[0]
    
    def get_force_range(self):
        return self.force_limits[1] - self.force_limits[0]
    
    def limit_width(self, current_width):
        if current_width < min(self.width_limits):
            current_width = min(self.width_limits)
        elif current_width > max(self.width_limits):
            current_width = max(self.width_limits)
        return current_width
    
    def limit_force(self, objective_force):
        if objective_force < min(self.force_limits):
            objective_force = min(self.force_limits)
        elif objective_force > max(self.force_limits):
            objective_force = max(self.force_limits)
        return objective_force