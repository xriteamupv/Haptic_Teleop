#!/usr/bin/env python3

class Mapper:
    pose_element = [0.0, 0.0, 0.0]

    @staticmethod
    def calc_slope(y1, m, x2, x1):
        return y1 + m*(x2-x1)
    
    @staticmethod
    def calc_non_linear(y1, a, b, x2, x1):
        return y1 + a*(x2 - x1)*(x2 - x1) + b*(x2 - x1)

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
        if robot_arm.mimic:
            pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
            pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        if robot_arm.inverted:
            pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])

        return pose_element
    
    @staticmethod
    def focused_linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]
        
        for coord in range(len(["x","y","z"])):
            min_foc_human_arm = 0.3*human_arm.max_limits[coord]
            max_foc_human_arm = 0.7*human_arm.max_limits[coord]
            if human_arm.max_limits[coord] == -human_arm.min_limits[coord]:
                slope_diff = [-0.2, -0.2, -0.05]
            else:
                slope_diff = [0.08, 0.08, -0.05]
            
            proportion_m1 = (robot_arm.max_limits[coord] - robot_arm.min_limits[coord]) / (human_arm.max_limits[coord] - human_arm.min_limits[coord])
            proportion_m2 = proportion_m1 + slope_diff[coord]
            
            if human_arm.pose_list[coord] < min_foc_human_arm:
                pose_element[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, human_arm.pose_list[coord], human_arm.min_limits[coord])
            elif min_foc_human_arm <= human_arm.pose_list[coord] <= max_foc_human_arm:
                x2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, human_arm.pose_list[coord], min_foc_human_arm)
                pose_element[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, human_arm.min_limits[coord])
            else:
                xm2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, max_foc_human_arm, min_foc_human_arm)
                proportion_m3 = (robot_arm.max_limits[coord] - Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, xm2,human_arm.min_limits[coord]))/(human_arm.max_limits[coord] - max_foc_human_arm)
                x3 = Mapper.calc_slope(max_foc_human_arm, proportion_m3/proportion_m2, human_arm.pose_list[coord], max_foc_human_arm)
                x2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, human_arm.min_limits[coord])

            pose_element[coord] = max(min(robot_arm.min_limits[coord], robot_arm.max_limits[coord]), min(max(robot_arm.min_limits[coord], robot_arm.max_limits[coord]), pose_element[coord]))

        #pose_element = robot_arm.limit_movement(pose_element)
        #pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        if robot_arm.mimic:
            pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
            pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        if robot_arm.inverted:
            pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])
        
        return pose_element
    
    @staticmethod
    def multi_focused_linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]

        for coord in range(len(["x","y","z"])):
            min_foc_human_arm = 0.15*human_arm.max_limits[coord]
            midmin_foc_human_arm = 0.35*human_arm.max_limits[coord]
            midmax_foc_human_arm = 0.65*human_arm.max_limits[coord]
            max_foc_human_arm = 0.85*human_arm.max_limits[coord]
            if human_arm.max_limits[coord] == -human_arm.min_limits[coord]:
                slopes_diff = [0.4, -0.2, -0.5, -0.2]
            else:
                slopes_diff = [0.3, -0.1, -0.2, -0.1]
            
            proportion_m1 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + slopes_diff[0] # m1
            proportion_m2 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + slopes_diff[1] # m2
            proportion_m3 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + slopes_diff[2] # m3
            proportion_m4 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + slopes_diff[3] # m4
            
            if human_arm.pose_list[coord] < min_foc_human_arm:
                pose_element.pose_list[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, human_arm.pose_list[coord], 0)
            elif min_foc_human_arm <= human_arm.pose_list[coord] < midmin_foc_human_arm:
                x2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, human_arm.pose_list[coord], min_foc_human_arm)
                pose_element.pose_list[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            elif midmin_foc_human_arm <= human_arm.pose_list[coord] < midmax_foc_human_arm:
                x3 = Mapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, human_arm.pose_list[coord], midmin_foc_human_arm)
                x2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element.pose_list[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            elif midmax_foc_human_arm <= human_arm.pose_list[coord] <= max_foc_human_arm:
                x4 = Mapper.calc_slope(midmax_foc_human_arm, proportion_m4/proportion_m3, human_arm.pose_list[coord], midmax_foc_human_arm)
                x3 = Mapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, x4, midmin_foc_human_arm)
                x2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element.pose_list[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)
            else:
                xm4 = Mapper.calc_slope(midmax_foc_human_arm, proportion_m4/proportion_m3, max_foc_human_arm, midmax_foc_human_arm)
                xm3 = Mapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, xm4, midmin_foc_human_arm)
                xm2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, xm3, min_foc_human_arm)
                proportion_m5 = (robot_arm.max_limits[coord] - Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, xm2,0))/(human_arm.max_limits[coord] - max_foc_human_arm)
                x5 = Mapper.calc_slope(max_foc_human_arm, proportion_m5/proportion_m4, human_arm.pose_list[coord], max_foc_human_arm)
                x4 = Mapper.calc_slope(midmax_foc_human_arm, proportion_m4/proportion_m3, x5, midmax_foc_human_arm)
                x3 = Mapper.calc_slope(midmin_foc_human_arm, proportion_m3/proportion_m2, x4, midmin_foc_human_arm)
                x2 = Mapper.calc_slope(min_foc_human_arm, proportion_m2/proportion_m1, x3, min_foc_human_arm)
                pose_element.pose_list[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, x2, 0)

        pose_element = robot_arm.limit_movement(pose_element)
        pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        if robot_arm.mimic:
            pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
            pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        if robot_arm.inverted:
            pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])
        
        return pose_element
    
    @staticmethod
    def non_linear_mapping_arm(human_arm, robot_arm):
        pose_element = [0.0, 0.0, 0.0]

        for coord in range(len(["x","y","z"])):
            min_foc_human_arm = 0.3*human_arm.max_limits[coord]
            max_foc_human_arm = 0.7*human_arm.max_limits[coord]
            if human_arm.max_limits[coord] == -human_arm.min_limits[coord]:
                non_linear_terms = [1.5, 0.05] # a, b
            else:
                non_linear_terms = [-1.5, 1.05] # a, b

            proportion_m1 = (human_arm.max_limits[coord]-human_arm.min_limits[coord])*(robot_arm.max_limits[coord]-robot_arm.min_limits[coord]) + 0.4 # m1
        
            if human_arm.pose_list[coord] < min_foc_human_arm:
                pose_element.pose_list[coord] = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, human_arm.pose_list[coord], 0)
            elif min_foc_human_arm <= human_arm.pose_list[coord] <= max_foc_human_arm:
                y2 = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, min_foc_human_arm, human_arm.min_limits[coord])
                pose_element.pose_list[coord] = Mapper.calc_non_linear(y2, non_linear_terms[0], non_linear_terms[1], human_arm.pose_list[coord], min_foc_human_arm)
            else:
                y2 = Mapper.calc_slope(robot_arm.min_limits[coord], proportion_m1, min_foc_human_arm, human_arm.min_limits[coord])
                y3 = Mapper.calc_non_linear(y2, non_linear_terms[0], non_linear_terms[1], max_foc_human_arm, min_foc_human_arm)
                proportion_m2 = (robot_arm.max_limits[coord] - y3)/(human_arm.max_limits[coord] - max_foc_human_arm)
                pose_element.pose_list[coord] = Mapper.calc_slope(y3, proportion_m2, human_arm.pose_list[coord], max_foc_human_arm)

        pose_element = robot_arm.limit_movement(pose_element)
        pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
        if robot_arm.mimic:
            pose_element[0] = robot_arm.max_limits[0] - (pose_element[0] - robot_arm.min_limits[0])
            pose_element[1] = robot_arm.max_limits[1] - (pose_element[1] - robot_arm.min_limits[1])
        if robot_arm.inverted:
            pose_element[2] = robot_arm.max_limits[2] - (pose_element[2] - robot_arm.min_limits[2])
        
        return pose_element