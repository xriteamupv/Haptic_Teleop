#!/usr/bin/env python3
import time
from utils.GripLimitator import GripLimitator
from utils.WidthMapper import WidthMapper
from utils.ForceMapper import ForceMapper

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
        self.widths_default = [99, 80, 60, 40, 0]
        self.limitator = GripLimitator(args)
        self.perform_model_adaptations(args)

    def perform_model_adaptations(self, args):
        self.poly_reg = None
        self.svr_model = None
        self.spline_model = None
        adapted_levels = range(args.grip_levels+1)
        adapted_widths = self.widths_default[0:(args.grip_levels+1)]
        if args.width_model == 3:
            self.poly_reg_model = WidthMapper.prep_poly_regression_width(adapted_levels,adapted_widths)
        elif args.width_model == 4:
            self.svm_model = WidthMapper.train_support_vector_machine_width(adapted_levels,adapted_widths)
        elif args.width_model == 5:
            self.spline_model = WidthMapper.prep_cubic_splin(adapted_levels,adapted_widths)

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
        if self.width_model == 5: # CUBIC SPLINE
            self.objective_width = self.grip_levels - WidthMapper.cubic_spline_width(grip_level, self.spline_model)
        elif self.width_model == 4: # SUPPORT VECTOR MACHINE
            self.objective_width = self.grip_levels - WidthMapper.predict_support_vector_machine_width(grip_level, self.svm_model)
        elif self.width_model == 3: # POLYNOMIAL REGRESSION
            self.objective_width = self.grip_levels - WidthMapper.polynomial_regression_width(grip_level, self.poly_reg_model)
        elif self.width_model == 2: # INTERVAL DETECTION LINEAR INTERPOLATION
            self.objective_width = self.grip_levels - WidthMapper.interval_detection_width(grip_level)
        elif self.width_model == 1: # LINEAR INTERPOLATION
            #self.objective_width = ((4-grip_level) * self.limitator.get_width_range()) / 4 # 4 grip levels
            self.objective_width = self.grip_levels - WidthMapper.linear_interpolation_width(grip_level)
        else: # DEFAULT
            self.objective_width = self.widths_default[grip_level]

    def configure_force(self, static_force, time_delta):
        time_diff_msec = round(float(time_delta.microseconds)*0.001, 3) 
        print("TIME_DIFF: ", time_diff_msec)
        if self.force_model == 5: # CUBIC SPLINE
            self.objective_force = ForceMapper.cubic_spline_force(time_diff_msec)
        elif self.force_model == 4: # PIECEWISE CUADRATIC
            self.objective_force = ForceMapper.piecewise_quadratic_mapping_force(time_diff_msec)
        elif self.force_model == 3: # EXPONENTIAL DECAY
            self.objective_force = ForceMapper.exponential_decay_force(time_diff_msec)
        elif self.force_model == 2: # LINEAR MAPPING
            self.objective_force = ForceMapper.linear_mapping_force(time_diff_msec)
        elif self.force_model == 1: # DYNAMIC FORCE
            if time_diff_msec > self.max_duration:
                self.objective_force = static_force
                print("STATIC FORCE: ", self.objective_force)
            else:
                self.objective_force = (time_diff_msec * self.limitator.get_force_range()) / self.max_duration # MAPEO DE FUERZA o EFFORT CON TIEMPO
                print("DYNAMIC FORCE: ", self.objective_force)
        else: # STATIC FORCE
            self.objective_force = static_force # default
    
    def configure_delay(self):
        if self.delay_model == 2:
            # DELAY MODEL GRIPPER
            c = 0
        elif self.delay_model == 1:
            max_time = self.time_max_width # seconds - time between min and max position
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
