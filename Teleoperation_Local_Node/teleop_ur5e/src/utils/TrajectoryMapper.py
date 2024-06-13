#!/usr/bin/env python3

class TrajectoryConditioner:

    def __init__(self, coord, mimic, inverted, robot_limits):

        if coord == 0: # x
            self.condition_x_axis(mimic, robot_limits)
        elif coord == 1: # y
            self.condition_y_axis(inverted, robot_limits)
        elif coord == 2: # z
            self.condition_z_axis(mimic, robot_limits)

    def condition_x_axis(self, mimic, diff_robot_limits):
        #self.min_limit_robot_arm = -0.4
        #self.max_limit_robot_arm = 0.4
        if not mimic:
            self.shift_val_linear = 0.0
            self.slope_diff_focused = - (diff_robot_limits)/4 
            self.slope_diffs_multifocused = [+0.15, -0.05, -0.25, -0.05]
            self.slope_diff_nonlinear = 0.4
            self.non_linear_terms = [0.9, 0.05, 0] # a 1, b 0.05
            self.shift_val_nonlinear = 0.0
        else:
            self.shift_val_linear = -0.12
            self.slope_diff_focused = - (diff_robot_limits)/4
            self.slope_diffs_multifocused = [+0.15, -0.05, -0.25, -0.05]
            self.slope_diff_nonlinear = 0.4
            self.non_linear_terms = [-1.4, 0.1, 0.4] # a 1, b 0.05
            self.shift_val_nonlinear = 0.5

    def condition_y_axis(self, mimic, diff_robot_limits):
        #self.min_limit_robot_arm = 0.740
        #self.max_limit_robot_arm = 0.500
        if not mimic:
            self.shift_val_linear = 0.0
            self.slope_diff_focused = - (diff_robot_limits)/5
            self.slope_diffs_multifocused = [+0.1, -0.0, -0.1, -0.0]
            self.slope_diff_nonlinear = -0.4
            self.non_linear_terms = [0.4, 0.1, -0.15] # a 1, b 0.05
            self.shift_val_nonlinear = 0.74
        else:
            self.shift_val_linear = -0.12
            self.slope_diff_focused = - (diff_robot_limits)/5
            self.slope_diffs_multifocused = [+0.1, -0.0, -0.1, -0.0]
            self.slope_diff_nonlinear = -0.4
            self.non_linear_terms = [-0.55, 0.18, -0.01] # a 1, b 0.05
            self.shift_val_nonlinear = -0.1

    def condition_z_axis(self, mimic, diff_robot_limits):
        #self.min_limit_robot_arm = 0.600
        #self.max_limit_robot_arm = 0.200
        if not mimic:
            self.shift_val_linear = 0.0
            self.slope_diff_focused = - (diff_robot_limits)/4
            self.slope_diffs_multifocused = [+0.1, -0.0, -0.1, -0.0]
            self.slope_diff_nonlinear = -0.7
            self.non_linear_terms = [0.6, 0.1, -0.26] # a 1, b 0.05
            self.shift_val_nonlinear = 0.74
        else:
            self.shift_val_linear = -0.12
            self.slope_diff_focused = - (diff_robot_limits)/4
            self.slope_diffs_multifocused = [+0.1, -0.0, -0.1, -0.0]
            self.slope_diff_nonlinear = -0.7
            self.non_linear_terms = [-0.7, 0.18, -0.01] # a 1, b 0.05
            self.shift_val_nonlinear = -0.1

class TrajectoryMapper:
    pose_element = [0.0, 0.0, 0.0]

    @staticmethod
    def calc_slope(y1, m, x2, x1):
        return y1 + m*(x2-x1)
    
    @staticmethod
    def calc_non_linear(y1, a, b, c, x2, x1):
        return y1 + c + a*(x2 - x1)*(x2 - x1) + b*(x2 - x1)

    @staticmethod
    def linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]
        proportion_element = 0.0
        
        # EJE HUMAN X, ROBOT X
        proportion_element = (human_arm.max_limits[0]-human_arm.min_limits[0])*(robot_arm.max_limits[0]-robot_arm.min_limits[0])
        pose_element[0] = robot_arm.min_limits[0] + proportion_element*human_arm.pose_list[0]

        # EJE HUMAN Y, ROBOT Z
        proportion_element = (human_arm.max_limits[1]-human_arm.min_limits[1])*(robot_arm.max_limits[2]-robot_arm.min_limits[2])
        pose_element[2] = robot_arm.min_limits[2] + proportion_element*human_arm.pose_list[1]

        # EJE HUMAN Z, ROBOT Y
        proportion_element = 500*(human_arm.max_limits[2]-human_arm.min_limits[2])*(robot_arm.max_limits[1]-robot_arm.min_limits[1])
        pose_element[1] = robot_arm.min_limits[1] + proportion_element*human_arm.pose_list[2]

        pose_element = robot_arm.limit_movement(pose_element)
        pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        if not robot_arm.mimic:
            pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
            pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        if not robot_arm.inverted:
            pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])

        return pose_element
    
    @staticmethod
    def focused_linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]
        
        for coord in range(len(["x","y","z"])):
            min_foc_human_arm = 0.3*human_arm.max_limits[coord]
            max_foc_human_arm = 0.7*human_arm.max_limits[coord]
            conditioner = TrajectoryConditioner(coord, robot_arm.mimic, robot_arm.inverted, robot_arm.max_limits[coord]-robot_arm.min_limits[coord])
            #if human_arm.max_limits[coord] == -human_arm.min_limits[coord]:
            #    slope_diff = [-0.2, -0.2, -0.05]
            #else:
            #    slope_diff = [0.08, 0.08, -0.05]
            
            proportion_m1 = (robot_arm.max_limits[coord] - robot_arm.min_limits[coord]) / (human_arm.max_limits[coord] - human_arm.min_limits[coord])
            proportion_m2 = proportion_m1 + conditioner.slope_diff_focused
            
            if human_arm.pose_list[coord] < min_foc_human_arm:
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, human_arm.pose_list[coord], human_arm.min_limits[coord])
            elif min_foc_human_arm <= human_arm.pose_list[coord] <= max_foc_human_arm:
                x2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, human_arm.pose_list[coord], min_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, human_arm.min_limits[coord])
            else:
                xm2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, max_foc_human_arm, min_foc_human_arm)
                proportion_m3 = (robot_arm.max_limits[coord] - TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, xm2,human_arm.min_limits[coord]))/(human_arm.max_limits[coord] - max_foc_human_arm)
                x3 = TrajectoryMapper.calc_slope(max_foc_human_arm, proportion_m3/proportion_m2, human_arm.pose_list[coord], max_foc_human_arm)
                x2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, human_arm.min_limits[coord])

            pose_element[coord] = max(min(robot_arm.min_limits[coord], robot_arm.max_limits[coord]), min(max(robot_arm.min_limits[coord], robot_arm.max_limits[coord]), pose_element[coord]))

        #pose_element = robot_arm.limit_movement(pose_element)
        #pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        #if not robot_arm.mimic:
        #    pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        #    pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        #if not robot_arm.inverted:
        #    pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])
        
        return pose_element
    
    @staticmethod
    def multi_focused_linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]

        for coord in range(len(["x","y","z"])):
            min_foc_human_arm = 0.15*human_arm.max_limits[coord]
            midmin_foc_human_arm = 0.35*human_arm.max_limits[coord]
            midmax_foc_human_arm = 0.65*human_arm.max_limits[coord]
            max_foc_human_arm = 0.85*human_arm.max_limits[coord]
            conditioner = TrajectoryConditioner(coord, robot_arm.mimic, robot_arm.inverted, robot_arm.max_limits[coord]-robot_arm.min_limits[coord])
            #if human_arm.max_limits[coord] == -human_arm.min_limits[coord]:
            #    slopes_diff = [0.4, -0.2, -0.5, -0.2]
            #else:
            #    slopes_diff = [0.3, -0.1, -0.2, -0.1]
            
            proportion_m1 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + conditioner.slope_diffs_multifocused[0] # m1
            proportion_m2 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + conditioner.slope_diffs_multifocused[1] # m2
            proportion_m3 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + conditioner.slope_diffs_multifocused[2] # m3
            proportion_m4 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + conditioner.slope_diffs_multifocused[3] # m4
            
            if human_arm.pose_list[coord] < min_foc_human_arm:
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, human_arm.pose_list[coord], 0)
            elif min_foc_human_arm <= human_arm.pose_list[coord] < midmin_foc_human_arm:
                x2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, human_arm.pose_list[coord], min_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            elif midmin_foc_human_arm <= human_arm.pose_list[coord] < midmax_foc_human_arm:
                x3 = TrajectoryMapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, human_arm.pose_list[coord], midmin_foc_human_arm)
                x2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            elif midmax_foc_human_arm <= human_arm.pose_list[coord] <= max_foc_human_arm:
                x4 = TrajectoryMapper.calc_slope(midmax_foc_human_arm, proportion_m4/proportion_m3, human_arm.pose_list[coord], midmax_foc_human_arm)
                x3 = TrajectoryMapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, x4, midmin_foc_human_arm)
                x2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            else:
                xm4 = TrajectoryMapper.calc_slope(midmax_foc_human_arm, proportion_m4/proportion_m3, max_foc_human_arm, midmax_foc_human_arm)
                xm3 = TrajectoryMapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, xm4, midmin_foc_human_arm)
                xm2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, xm3, min_foc_human_arm)
                proportion_m5 = (robot_arm.max_limits[coord] - TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, xm2,0))/(human_arm.max_limits[coord] - max_foc_human_arm)
                x5 = TrajectoryMapper.calc_slope(max_foc_human_arm, proportion_m5/proportion_m4, human_arm.pose_list[coord], max_foc_human_arm)
                x4 = TrajectoryMapper.calc_slope(midmax_foc_human_arm, proportion_m4/proportion_m3, x5, midmax_foc_human_arm)
                x3 = TrajectoryMapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, x4, midmin_foc_human_arm)
                x2 = TrajectoryMapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            
            pose_element[coord] = max(min(robot_arm.min_limits[coord], robot_arm.max_limits[coord]), min(max(robot_arm.min_limits[coord], robot_arm.max_limits[coord]), pose_element[coord]))
        
        #if not robot_arm.mimic:
        #    pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        #    pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        #if not robot_arm.inverted:
        #    pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])

        #pose_element = robot_arm.limit_movement(pose_element)
        #pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        #if not robot_arm.mimic:
        #    pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        #    pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        #if not robot_arm.inverted:
        #    pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])
        
        return pose_element
    
    @staticmethod
    def non_linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]

        for coord in range(len(["x","y","z"])):
            min_foc_human_arm = 0.3*human_arm.max_limits[coord]
            max_foc_human_arm = 0.7*human_arm.max_limits[coord]
            conditioner = TrajectoryConditioner(coord, robot_arm.mimic, robot_arm.inverted, robot_arm.max_limits[coord]-robot_arm.min_limits[coord])
            #if human_arm.max_limits[coord] == -human_arm.min_limits[coord]:
            #    non_linear_terms = [1.5, 0.05] # a, b
            #else:
            #    non_linear_terms = [-1.5, 1.05] # a, b

            proportion_m1 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + conditioner.slope_diff_nonlinear # m1
        
            if human_arm.pose_list[coord] < min_foc_human_arm:
                pose_element[coord] = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, human_arm.pose_list[coord], 0)
            elif min_foc_human_arm <= human_arm.pose_list[coord] <= max_foc_human_arm:
                y2 = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, min_foc_human_arm, human_arm.min_limits[coord])
                pose_element[coord] = TrajectoryMapper.calc_non_linear(y2, conditioner.non_linear_terms[0], conditioner.non_linear_terms[1], conditioner.non_linear_terms[2], human_arm.pose_list[coord], min_foc_human_arm + conditioner.shift_val_nonlinear)
            else:
                y2 = TrajectoryMapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, min_foc_human_arm, human_arm.min_limits[coord])
                y3 = TrajectoryMapper.calc_non_linear(y2, conditioner.non_linear_terms[0], conditioner.non_linear_terms[1], conditioner.non_linear_terms[2], max_foc_human_arm, min_foc_human_arm + conditioner.shift_val_nonlinear)
                proportion_m2 = (robot_arm.max_limits[coord] - y3)/(human_arm.max_limits[coord] - max_foc_human_arm)
                pose_element[coord] = TrajectoryMapper.calc_slope(y3, proportion_m2, human_arm.pose_list[coord], max_foc_human_arm)

            if robot_arm.min_limits[coord] < robot_arm.max_limits[coord]:
                pose_element[coord] = max(robot_arm.min_limits[coord], min(robot_arm.max_limits[coord], pose_element[coord]))
            else:
                pose_element[coord] = max(robot_arm.max_limits[coord], min(robot_arm.min_limits[coord], pose_element[coord]))

        #if not robot_arm.mimic:
        #    pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        #    pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        #if not robot_arm.inverted:
        #    pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])
            
        #pose_element = robot_arm.limit_movement(pose_element)
        #pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        
        return pose_element