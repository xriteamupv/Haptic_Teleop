import math
import numpy as np
from scipy.interpolate import CubicSpline
from sklearn.svm import SVR
import matplotlib.pyplot as plt

def map_duration_to_force1(duration, min_duration, max_duration, min_force=0, max_force=70):
    """
    Maps the temporal duration to an objective force for a robotic gripper.

    :param duration: Duration in milliseconds or microseconds.
    :param min_duration: Minimum expected duration.
    :param max_duration: Maximum expected duration.
    :param min_force: Minimum force (default 0 Newtons).
    :param max_force: Maximum force (default 70 Newtons).
    :return: Calculated force in Newtons.
    """
    # Ensure the duration is within the expected range
    if duration < min_duration:
        duration = min_duration
    elif duration > max_duration:
        duration = max_duration

    # Linear mapping from duration to force
    force = min_force + (max_force - min_force) * (max_duration - duration) / (max_duration - min_duration)
    
    return force

# Example usage
min_duration = 1000  # Minimum duration in milliseconds (1 second)
max_duration = 10000  # Maximum duration in milliseconds (10 seconds)
duration = 5000  # Example duration in milliseconds (5 seconds)

force = map_duration_to_force1(duration, min_duration, max_duration)
print(f"Calculated force: {force} Newtons")

def map_duration_to_force_exponential(duration, min_duration, max_duration, min_force=0, max_force=70, decay_rate=5):
    """
    Maps the temporal duration to an objective force for a robotic gripper using an exponential decay function.

    :param duration: Duration in milliseconds or microseconds.
    :param min_duration: Minimum expected duration.
    :param max_duration: Maximum expected duration.
    :param min_force: Minimum force (default 0 Newtons).
    :param max_force: Maximum force (default 70 Newtons).
    :param decay_rate: Rate of exponential decay (higher value means quicker drop in force).
    :return: Calculated force in Newtons.
    """
    # Ensure the duration is within the expected range
    duration = max(min_duration, min(max_duration, duration))

    # Normalize duration to a range between 0 and 1
    normalized_duration = (duration - min_duration) / (max_duration - min_duration)
    
    # Apply exponential decay function
    force = min_force + (max_force - min_force) * math.exp(-decay_rate * normalized_duration)
    
    return force

# Example usage
min_duration = 1000  # Minimum duration in milliseconds (1 second)
max_duration = 10000  # Maximum duration in milliseconds (10 seconds)
duration = 5000  # Example duration in milliseconds (5 seconds)
decay_rate = 2  # Adjust this value to control the rate of decay

force = map_duration_to_force_exponential(duration, min_duration, max_duration, decay_rate=decay_rate)
print(f"Calculated force: {force:.2f} Newtons")

def map_duration_to_force_piecewise(duration, min_duration, max_duration, min_force=0, max_force=70):
    """
    Maps the temporal duration to an objective force for a robotic gripper using a piecewise quadratic function.

    :param duration: Duration in milliseconds or microseconds.
    :param min_duration: Minimum expected duration.
    :param max_duration: Maximum expected duration.
    :param min_force: Minimum force (default 0 Newtons).
    :param max_force: Maximum force (default 70 Newtons).
    :return: Calculated force in Newtons.
    """
    # Define the breakpoints for the piecewise function
    breakpoints = [min_duration, (min_duration + max_duration) / 2, max_duration]
    
    # Ensure the duration is within the expected range
    duration = max(min_duration, min(max_duration, duration))
    
    # Calculate normalized duration within each segment
    if duration <= breakpoints[1]:
        # First segment: Quadratic decrease
        normalized_duration = (duration - min_duration) / (breakpoints[1] - min_duration)
        force = max_force - (max_force - (max_force + min_force) / 2) * normalized_duration**2
    else:
        # Second segment: Quadratic decrease
        normalized_duration = (duration - breakpoints[1]) / (max_duration - breakpoints[1])
        force = (max_force + min_force) / 2 - ((max_force + min_force) / 2 - min_force) * normalized_duration**2
    
    return force

# Example usage
min_duration = 1000  # Minimum duration in milliseconds (1 second)
max_duration = 10000  # Maximum duration in milliseconds (10 seconds)
duration = 5000  # Example duration in milliseconds (5 seconds)

force = map_duration_to_force_piecewise(duration, min_duration, max_duration)
print(f"Calculated force PIECEWISE: {force:.2f} Newtons")

def map_duration_to_force_spline(duration, min_duration, max_duration, min_force=0, max_force=70):
    """
    Maps the temporal duration to an objective force for a robotic gripper using cubic spline interpolation.

    :param duration: Duration in milliseconds or microseconds.
    :param min_duration: Minimum expected duration.
    :param max_duration: Maximum expected duration.
    :param min_force: Minimum force (default 0 Newtons).
    :param max_force: Maximum force (default 70 Newtons).
    :return: Calculated force in Newtons.
    """
    # Define key points for duration and corresponding forces
    duration_points = np.array([min_duration, (min_duration + max_duration) / 3, (min_duration + max_duration) * 2 / 3, max_duration])
    force_points = np.array([max_force, max_force / 2, max_force / 4, min_force])
    
    # Create a cubic spline interpolation
    spline = CubicSpline(duration_points, force_points)
    
    # Ensure the duration is within the expected range
    duration = max(min_duration, min(max_duration, duration))
    
    # Calculate the force using the spline interpolation
    force = spline(duration)
    
    return force

# Example usage
min_duration = 1000  # Minimum duration in milliseconds (1 second)
max_duration = 10000  # Maximum duration in milliseconds (10 seconds)
duration = 5000  # Example duration in milliseconds (5 seconds)

force = map_duration_to_force_spline(duration, min_duration, max_duration)
print(f"Calculated force: {force:.2f} Newtons")

# Plotting for visualization
#durations = np.linspace(min_duration, max_duration, 1000)
#forces = [map_duration_to_force_spline(d, min_duration, max_duration) for d in durations]

#plt.plot(durations, forces)
#plt.xlabel('Duration (ms)')
#plt.ylabel('Force (N)')
#plt.title('Duration to Force Mapping Using Spline Interpolation')
#plt.grid(True)
#plt.show()


dur_range = range(min_duration, max_duration+1)
grip_force1 = []
grip_force2 = []
grip_force3 = []
grip_force4 = []
for duration in dur_range:
    #print("LVL: ", grip_level)
    grip_force1.append(map_duration_to_force1(duration, min_duration, max_duration))
    #print(f"Grip Force: {grip_force1[end]}, Duration: {duration}")
    grip_force2.append(map_duration_to_force_exponential(duration, min_duration, max_duration))
    #print(f"Grip Level: {grip_level}, Objective Width: {grip_width1}")
    #grip_width3.append(poly_regression_width(grip_level/100))
    grip_force3.append(map_duration_to_force_piecewise(duration, min_duration, max_duration))
    #print(f"Grip Level: {grip_level}, Objective Width: {grip_width1}")
    grip_force4.append(map_duration_to_force_spline(duration, min_duration, max_duration))
    #print(f"Grip Level: {grip_level}, Objective Width: {grip_width1}")

fig, axs = plt.subplots(2, 2, layout='constrained')
axs[0][0].plot(dur_range, grip_force1)
axs[0][0].set_xlabel('Duration (milliseconds)')
axs[0][0].set_ylabel('Grip Force (Newtons)')
axs[0][0].grid(True)

axs[0][1].plot(dur_range, grip_force2)
axs[0][1].set_xlabel('Duration (milliseconds)')
axs[0][1].set_ylabel('Grip Force (Newtons)')
axs[0][1].grid(True)

axs[1][0].plot(dur_range, grip_force3)
axs[1][0].set_xlabel('Duration (milliseconds)')
axs[1][0].set_ylabel('Grip Force (Newtons)')
axs[1][0].grid(True)

axs[1][1].plot(dur_range, grip_force4)
axs[1][1].set_xlabel('Duration (milliseconds)')
axs[1][1].set_ylabel('Grip Force (Newtons)')
axs[1][1].grid(True)

plt.show()