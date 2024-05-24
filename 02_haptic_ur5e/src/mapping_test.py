
import matplotlib.pyplot as plt
import numpy as np

def calc_slope(y1, m, x2, x1):
    return y1 + m*(x2-x1)

def calc_non_linear(y1, a, b, x2, x1):
    return y1 + a*(x2 - x1)*(x2 - x1) + b*(x2 - x1)

if __name__ == "__main__":

    min_limit_robot_arm = 0.600
    max_limit_robot_arm = 0.200
    min_limit_human_arm = 0
    max_limit_human_arm = 1
    mimic = False

    #if mimic:
    #    min_limit_robot_arm = 0.400
    #    max_limit_robot_arm = -0.400

    lin_pose_human_arm = np.linspace(min_limit_human_arm,max_limit_human_arm,101)
    lin_pose_robot_arm = [0 for i in range(len(lin_pose_human_arm))]

    for i, pose_arm in enumerate(lin_pose_human_arm):
        proportion = abs(max_limit_human_arm-min_limit_human_arm)*(max_limit_robot_arm-min_limit_robot_arm)
        lin_pose_robot_arm[i] = min_limit_robot_arm + proportion*pose_arm
        if mimic:
            lin_pose_robot_arm[i] = max_limit_robot_arm - (lin_pose_robot_arm[i] - min_limit_robot_arm)
            #lin_pose_robot_arm[i] = lin_pose_robot_arm[i] - 2*(proportion*pose_arm - max_limit_robot_arm)

    fig, axs = plt.subplots(2, 2, layout='constrained')
    axs[0][0].plot(lin_pose_human_arm, lin_pose_robot_arm)
    axs[0][0].set_xlabel('Human Arm Coordinates')
    axs[0][0].set_ylabel('Robot Arm Coordinates')
    axs[0][0].grid(True)


    #min_foc_human_arm = 0.3*max_limit_human_arm
    #max_foc_human_arm = 0.7*max_limit_human_arm
    #lin_foc_pose_human_arm = np.linspace(min_limit_human_arm,max_limit_human_arm,101)
    #lin_foc_pose_robot_arm = [0 for i in range(len(lin_foc_pose_human_arm))]

    #proportion_m1 = (max_limit_human_arm-min_limit_human_arm)*(max_limit_robot_arm-min_limit_robot_arm) + 0.4 # m1
    #proportion_m2 = (max_limit_human_arm-min_limit_human_arm)*(max_limit_robot_arm-min_limit_robot_arm) - 0.6 # m2
    #proportion_m3 = (max_limit_robot_arm - min_limit_robot_arm - proportion_m1*min_foc_human_arm - proportion_m2*(max_foc_human_arm-min_foc_human_arm))/(max_limit_human_arm - max_foc_human_arm) #(max_limit_human_arm-min_limit_human_arm)*(max_limit_robot_arm-min_limit_robot_arm) + 1 # m3

    #for i, pose_arm in enumerate(lin_foc_pose_human_arm):
    #    if pose_arm < min_foc_human_arm:
    #        lin_foc_pose_robot_arm[i] = min_limit_robot_arm + proportion_m1*pose_arm
    #    elif min_foc_human_arm <= pose_arm <= max_foc_human_arm:
    #        lin_foc_pose_robot_arm[i] = min_limit_robot_arm + proportion_m1*min_foc_human_arm + proportion_m2*(pose_arm-min_foc_human_arm)
    #    else:
    #        #print("Entro")
    #        lin_foc_pose_robot_arm[i] = min_limit_robot_arm + proportion_m1*min_foc_human_arm + proportion_m2*(max_foc_human_arm-min_foc_human_arm) + proportion_m3*(pose_arm-max_foc_human_arm)
    #    if lin_foc_pose_robot_arm[i] < min_limit_robot_arm:
    #        lin_foc_pose_robot_arm[i] = min_limit_robot_arm
    #    if lin_foc_pose_robot_arm[i] > max_limit_robot_arm:
    #        lin_foc_pose_robot_arm[i] = max_limit_robot_arm

    min_foc_human_arm = 0.3*max_limit_human_arm
    max_foc_human_arm = 0.7*max_limit_human_arm
    lin_foc_pose_human_arm = np.linspace(min_limit_human_arm,max_limit_human_arm,101)
    lin_foc_pose_robot_arm = [0 for i in range(len(lin_foc_pose_human_arm))]

    proportion_m1 = (max_limit_robot_arm - min_limit_robot_arm) / (max_limit_human_arm - min_limit_human_arm)
    print("PROPORTION: ", proportion_m1)
    proportion_m2 = proportion_m1 - 0.05 # Sym: - 0.2 #(max_limit_robot_arm - min_limit_robot_arm)/2  # Adjusted to ensure smooth transition within the focal range

    for i, pose_arm in enumerate(lin_foc_pose_human_arm):
        if pose_arm < min_foc_human_arm:
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, pose_arm, min_limit_human_arm)
            if mimic:
                print("MIMIC")
                lin_foc_pose_robot_arm[i] = max_limit_robot_arm - (lin_foc_pose_robot_arm[i] - min_limit_robot_arm)
                #lin_foc_pose_robot_arm[i] -= 2 * (proportion_m1 * (pose_arm - min_limit_human_arm) + min_limit_robot_arm)
        elif min_foc_human_arm <= pose_arm <= max_foc_human_arm:
            x2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, pose_arm, min_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, x2, min_limit_human_arm)
            if mimic:
                lin_foc_pose_robot_arm[i] = max_limit_robot_arm - (lin_foc_pose_robot_arm[i] - min_limit_robot_arm)
                #lin_foc_pose_robot_arm[i] = -lin_foc_pose_robot_arm[i]
        else:
            xm2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, max_foc_human_arm, min_foc_human_arm)
            proportion_m3 = (max_limit_robot_arm - calc_slope(min_limit_robot_arm, proportion_m1, xm2, min_limit_human_arm)) / (max_limit_human_arm - max_foc_human_arm)
            x3 = calc_slope(max_foc_human_arm, proportion_m3 / proportion_m2, pose_arm, max_foc_human_arm)
            x2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, x3, min_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, x2, min_limit_human_arm)
            if mimic:
                lin_foc_pose_robot_arm[i] = max_limit_robot_arm - (lin_foc_pose_robot_arm[i] - min_limit_robot_arm)
                #lin_foc_pose_robot_arm[i] = -lin_foc_pose_robot_arm[i]

        lin_foc_pose_robot_arm[i] = max(min(min_limit_robot_arm, max_limit_robot_arm), min(max(min_limit_robot_arm, max_limit_robot_arm), lin_foc_pose_robot_arm[i]))
        #lin_foc_pose_robot_arm[i] = max(min_limit_robot_arm, min(max_limit_robot_arm, lin_foc_pose_robot_arm[i]))

    axs[0][1].plot(lin_foc_pose_human_arm, lin_foc_pose_robot_arm)
    axs[0][1].set_xlabel('Human Arm Coordinates')
    axs[0][1].set_ylabel('Robot Arm Coordinates')
    axs[0][1].grid(True)

    min_foc_human_arm = 0.15*max_limit_human_arm
    midmin_foc_human_arm = 0.35*max_limit_human_arm
    midmax_foc_human_arm = 0.65*max_limit_human_arm
    max_foc_human_arm = 0.85*max_limit_human_arm
    lin_foc_pose_human_arm = np.linspace(min_limit_human_arm, max_limit_human_arm, 101)
    lin_foc_pose_robot_arm = [0 for _ in range(len(lin_foc_pose_human_arm))]

    proportion_m1 = (max_limit_robot_arm - min_limit_robot_arm) / (max_limit_human_arm - min_limit_human_arm) + 0.1 # Asym: + 0.3 # m1
    proportion_m2 = (max_limit_robot_arm - min_limit_robot_arm) / (max_limit_human_arm - min_limit_human_arm) - 0.05 # Asym: - 0.1 # m2
    proportion_m3 = (max_limit_robot_arm - min_limit_robot_arm) / (max_limit_human_arm - min_limit_human_arm) - 0.1 # Asym: - 0.2 # m3
    proportion_m4 = (max_limit_robot_arm - min_limit_robot_arm) / (max_limit_human_arm - min_limit_human_arm) - 0.05 # Asym: - 0.1 # m4

    for i, pose_arm in enumerate(lin_foc_pose_human_arm):
        if pose_arm < min_foc_human_arm:
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, pose_arm, 0)
        elif min_foc_human_arm <= pose_arm < midmin_foc_human_arm:
            x2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, pose_arm, min_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, x2, 0)
        elif midmin_foc_human_arm <= pose_arm < midmax_foc_human_arm:
            x3 = calc_slope(midmin_foc_human_arm, proportion_m3 / proportion_m2, pose_arm, midmin_foc_human_arm)
            x2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, x3, min_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, x2, 0)
        elif midmax_foc_human_arm <= pose_arm <= max_foc_human_arm:
            x4 = calc_slope(midmax_foc_human_arm, proportion_m4 / proportion_m3, pose_arm, midmax_foc_human_arm)
            x3 = calc_slope(midmin_foc_human_arm, proportion_m3 / proportion_m2, x4, midmin_foc_human_arm)
            x2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, x3, min_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, x2, 0)
        else:
            xm4 = calc_slope(midmax_foc_human_arm, proportion_m4 / proportion_m3, max_foc_human_arm, midmax_foc_human_arm)
            xm3 = calc_slope(midmin_foc_human_arm, proportion_m3 / proportion_m2, xm4, midmin_foc_human_arm)
            xm2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, xm3, min_foc_human_arm)
            proportion_m5 = (max_limit_robot_arm - calc_slope(min_limit_robot_arm, proportion_m1, xm2, 0)) / (max_limit_human_arm - max_foc_human_arm)
            x5 = calc_slope(max_foc_human_arm, proportion_m5 / proportion_m4, pose_arm, max_foc_human_arm)
            x4 = calc_slope(midmax_foc_human_arm, proportion_m4 / proportion_m3, x5, midmax_foc_human_arm)
            x3 = calc_slope(midmin_foc_human_arm, proportion_m3 / proportion_m2, x4, midmin_foc_human_arm)
            x2 = calc_slope(min_foc_human_arm, proportion_m2 / proportion_m1, x3, min_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, x2, 0)
        if mimic:
                lin_foc_pose_robot_arm[i] = max_limit_robot_arm - (lin_foc_pose_robot_arm[i] - min_limit_robot_arm)

        lin_foc_pose_robot_arm[i] = max(min(min_limit_robot_arm, max_limit_robot_arm), min(max(min_limit_robot_arm, max_limit_robot_arm), lin_foc_pose_robot_arm[i]))

    axs[1][0].plot(lin_foc_pose_human_arm, lin_foc_pose_robot_arm)
    axs[1][0].set_xlabel('Human Arm Coordinates')
    axs[1][0].set_ylabel('Robot Arm Coordinates')
    axs[1][0].grid(True)

    min_foc_human_arm = 0.3*max_limit_human_arm
    max_foc_human_arm = 0.7*max_limit_human_arm
    non_linear_terms = [-1.5, 1.05] # Symmetric [1.5, 0.05] # a, b
    lin_foc_pose_human_arm = np.linspace(min_limit_human_arm,max_limit_human_arm,101)
    lin_foc_pose_robot_arm = [0 for i in range(len(lin_foc_pose_human_arm))]

    proportion_m1 = (max_limit_human_arm-min_limit_human_arm)*(max_limit_robot_arm-min_limit_robot_arm) + 0.4 # m1
    
    for i, pose_arm in enumerate(lin_foc_pose_human_arm):
        if pose_arm < min_foc_human_arm:
            lin_foc_pose_robot_arm[i] = calc_slope(min_limit_robot_arm, proportion_m1, pose_arm, 0)
        elif min_foc_human_arm <= pose_arm <= max_foc_human_arm:
            y2 = calc_slope(min_limit_robot_arm, proportion_m1, min_foc_human_arm, min_limit_human_arm)
            lin_foc_pose_robot_arm[i] = calc_non_linear(y2, non_linear_terms[0], non_linear_terms[1], pose_arm, min_foc_human_arm)
        else:
            y2 = calc_slope(min_limit_robot_arm, proportion_m1, min_foc_human_arm, min_limit_human_arm)
            y3 = calc_non_linear(y2, non_linear_terms[0], non_linear_terms[1], max_foc_human_arm, min_foc_human_arm)
            proportion_m2 = (max_limit_robot_arm - y3) / (max_limit_human_arm - max_foc_human_arm)
            lin_foc_pose_robot_arm[i] = calc_slope(y3, proportion_m2, pose_arm, max_foc_human_arm)

        lin_foc_pose_robot_arm[i] = max(min_limit_robot_arm, min(max_limit_robot_arm, lin_foc_pose_robot_arm[i]))

        if mimic:
            lin_foc_pose_robot_arm[i] = max_limit_robot_arm - (lin_foc_pose_robot_arm[i] - min_limit_robot_arm)

    axs[1][1].plot(lin_foc_pose_human_arm, lin_foc_pose_robot_arm)
    axs[1][1].set_xlabel('Human Arm Coordinates')
    axs[1][1].set_ylabel('Robot Arm Coordinates')
    axs[1][1].grid(True)

    plt.show()