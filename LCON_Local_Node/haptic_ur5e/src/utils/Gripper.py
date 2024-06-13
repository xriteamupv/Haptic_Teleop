#!/usr/bin/env python3
import time
from utils.GripLimitator import GripLimitator

class Gripper():

    def __init__(self, args):
        self.grip_levels = args.grip_levels # Default 4
        self.width_model = args.width_model
        self.force_model = args.force_model
        self.delay_model = args.delay_model
        self.current_width = args.start_width # open
        self.width_tolerance = args.width_tolerance # 2
        self.force_tolerance = args.force_tolerance # 2
        self.max_duration = args.max_duration # 60.0 msec
        self.time_max_width = args.time_max_width # 2 sec
        self.limitator = GripLimitator(args)

    def on_open_or_close(self):
        self.objective_width = 99.0
        self.objective_force = 30.0
        self.prev_objective_width = 0.0

    def is_within_range(self, value, reference, tolerance):
        return value >= reference - tolerance and value <= reference + tolerance

    def is_movement_detected(self):
        return not self.is_within_range(self.current_width, self.objective_width, self.width_tolerance) and self.objective_force != 0.0
    
    def is_width_correct(self, val_width):
        return val_width >= self.limitator.get_min_width() and val_width <= self.limitator.get_max_width()
    
    def update_width(self, new_width):
        self.current_width = new_width

    def grip_within_limits(self):
        width_within_limits = self.is_within_range(self.current_width, self.limitator.get_min_width(), self.width_tolerance) or self.is_within_range(self.current_width, self.limitator.get_max_width(), self.width_tolerance)
        force_within_limits = self.is_within_range(self.objective_force, self.limitator.get_min_force(), self.force_tolerance) or self.is_within_range(self.objective_force, self.limitator.get_max_force(), self.force_tolerance)
        return width_within_limits and force_within_limits

    def get_width_variation(self):
        return abs(self.objective_width - self.current_width)
    
    def configure_width(self, grip_level):
        if self.width_model == 2:
            # GRIP MAPPER LEVEL WIDTH
            c = 0
        elif self.width_model == 1:
            self.objective_width = ((4-grip_level) * self.limitator.get_width_range()) / 4 # 4 grip levels
        else:
            if grip_level == 0:
                self.objective_width = 99.0
            elif grip_level == 1:
                self.objective_width = 80.0
            elif grip_level == 2:
                self.objective_width = 60.0
            elif grip_level == 3:
                self.objective_width = 40.0
            elif grip_level == 4:
                self.objective_width = 0.0

    def configure_force(self, static_force, time_delta):
        #if self.force_model in (1,2):
        time_diff_msec = round(float(time_delta.microseconds)*0.001, 3) 
        print("TIME_DIFF: ", time_diff_msec)
        if self.force_model == 2:
            #GRIP MAPPER FORCE TIME
            c = 0
        elif self.force_model == 1:
            if time_diff_msec > self.max_duration:
                self.objective_force = static_force
                print("STATIC FORCE: ", self.objective_force)
            else:
                self.objective_force = (time_diff_msec * self.limitator.get_force_range()) / self.max_duration # MAPEO DE FUERZA o EFFORT CON TIEMPO
                print("DYNAMIC FORCE: ", self.objective_force)
        else:
            self.objective_force = static_force # default
    
    def configure_delay(self):
        if self.delay_model == 2:
            # DELAY MODEL GRIPPER
            c = 0
        elif self.delay_model == 1:
            max_time = self.time_max_width # seconds - time between min and max posicion
            time_required = (self.get_width_variation() * max_time) / self.get_max_width_difference()
            time.sleep(time_required)
        else:
            time.sleep(self.time_max_width)

    def limit_movement(self):
        self.current_width = self.limitator.limit_width(self.current_width)
        self.objective_force = self.limitator.limit_force(self.objective_force)

    def limit_grip_level(self, grip_level):
        if grip_level > self.grip_levels:
            grip_level = self.grip_levels
        return grip_level

    def get_max_width_difference(self):
        return abs(self.limitator.get_width_range())

    def same_objective_position(self):
        return self.objective_width == self.prev_objective_width
    
    def update_objective_position(self):
        self.prev_objective_width = self.objective_width