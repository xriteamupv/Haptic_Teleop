import numpy as np
from numpy.polynomial.polynomial import Polynomial
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def linear_interpolation_width(grip_level):
    # Check if grip_level is within the valid range
    if not (0 <= grip_level <= 4):
        raise ValueError("Grip level must be between 0 and 4 inclusive.")

    # Map the grip level (0-4) to the objective width (0-99)
    # Using linear interpolation formula: y = ((y2 - y1) / (x2 - x1)) * (x - x1) + y1
    # Here, x1 = 0, x2 = 4, y1 = 0, y2 = 99
    objective_width = (99 / 4) * grip_level

    return int(objective_width)

def interval_decision_width(grip_level):
    if not (0 <= grip_level <= 4):
        raise ValueError("Grip level must be between 0 and 4 inclusive.")
    
    # Define breakpoints for grip levels and corresponding widths
    breakpoints = [0, 1, 2, 3, 4]
    widths = [0, 20, 50, 80, 99]
    
    # Find the interval that the grip level falls into
    for i in range(1, len(breakpoints)):
        if grip_level <= breakpoints[i]:
            # Perform linear interpolation within the interval
            x1, x2 = breakpoints[i - 1], breakpoints[i]
            y1, y2 = widths[i - 1], widths[i]
            width = ((y2 - y1) / (x2 - x1)) * (grip_level - x1) + y1
            return int(width)
    
    # Fallback in case the above logic doesn't work as expected
    return widths[-1]

def poly_regression_width(grip_level):
    
    # Define the grip levels and corresponding objective widths
    grip_levels = np.array([0, 1, 2, 3, 4])
    objective_widths = np.array([0, 20, 50, 80, 99])

    # Fit a polynomial regression model
    degree = 5  # Degree of the polynomial
    coefficients = np.polyfit(grip_levels, objective_widths, degree)

    # Create a polynomial object using the fitted coefficients
    polynomial = Polynomial(coefficients[::-1])

    # Use the polynomial model to predict the objective width
    objective_width = polynomial(grip_level)
    
    # Ensure the objective width is within the valid range (0-99)
    objective_width = max(0, min(99, objective_width))
    
    return int(objective_width)

def cubic_spline_width(grip_level):
    
    grip_levels = np.array([0, 1, 2, 3, 4])
    objective_widths = np.array([0, 20, 50, 80, 99])

    spline = CubicSpline(grip_levels, objective_widths, bc_type='natural')
    objective_width = spline(grip_level)
    
    # Ensure the objective width is within the valid range (0-99)
    objective_width = max(0, min(99, objective_width))
    
    return int(objective_width)

# Example usage
#for grip_level in range(5):
#    print(f"Grip Level: {grip_level}, Objective Width: {map_grip_to_width4(grip_level)}")

# Define the grip levels and corresponding objective widths
grip_levels = np.array([0, 1, 2, 3, 4]).reshape(-1, 1)
objective_widths = np.array([0, 20, 50, 80, 99])

# Train an SVR model
svr = SVR(kernel='rbf', C=100, gamma=0.1, epsilon=0.1)
svr.fit(grip_levels, objective_widths)

def map_grip_to_width(grip_level):
    if not (0 <= grip_level <= 4):
        raise ValueError("Grip level must be between 0 and 4 inclusive.")
    
    # Use the SVR model to predict the objective width
    objective_width = svr.predict(np.array([[grip_level]]))[0]
    
    # Ensure the objective width is within the valid range (0-99)
    objective_width = max(0, min(99, objective_width))
    
    return int(objective_width)

# Example usage
#for grip_level in range(5):
#    print(f"Grip Level: {grip_level}, Objective Width: {map_grip_to_width(grip_level)}")

# Visualization (optional)
#grip_levels_range = np.linspace(0, 4, 100).reshape(-1, 1)
#predicted_widths = svr.predict(grip_levels_range)

#plt.scatter(grip_levels, objective_widths, color='red', label='Data Points')
#plt.plot(grip_levels_range, predicted_widths, color='blue', label='SVR Model')
#plt.xlabel('Grip Level')
#plt.ylabel('Objective Width')
#plt.legend()
#plt.show()

grip_lvls = range(401)
widths_default = [0, 40, 60, 80, 99] #[99, 80, 60, 40, 0]
grip_width1 = []
grip_width2 = []
grip_width3 = []
grip_width4 = []
for grip_level in grip_lvls:
    #print("LVL: ", grip_level)
    grip_width1.append(linear_interpolation_width(grip_level/100))
    print(f"Grip Level: {grip_level/100}, Objective Width: {grip_width1}")
    grip_width2.append(interval_decision_width(grip_level/100))
    #print(f"Grip Level: {grip_level}, Objective Width: {grip_width1}")
    #grip_width3.append(poly_regression_width(grip_level/100))
    grip_width3.append(widths_default[int(grip_level/100)])
    #print(f"Grip Level: {grip_level}, Objective Width: {grip_width1}")
    grip_width4.append(cubic_spline_width(grip_level/100))
    #print(f"Grip Level: {grip_level}, Objective Width: {grip_width1}")

fig, axs = plt.subplots(2, 2, layout='constrained')
grip_levels = [4-lvl/100 for lvl in grip_lvls]
axs[0][0].plot(grip_levels, grip_width1)
axs[0][0].set_xlabel('Grip Level (open/soft/med/hard/close)')
axs[0][0].set_ylabel('Grip Width (millimeters)')
axs[0][0].grid(True)

axs[0][1].plot(grip_levels, grip_width2)
axs[0][1].set_xlabel('Grip Level (open/soft/med/hard/close)')
axs[0][1].set_ylabel('Grip Width (millimeters)')
axs[0][1].grid(True)

axs[1][0].plot(grip_levels, grip_width3)
axs[1][0].set_xlabel('Grip Level (open/soft/med/hard/close)')
axs[1][0].set_ylabel('Grip Width (millimeters)')
axs[1][0].grid(True)

axs[1][1].plot(grip_levels, grip_width4)
axs[1][1].set_xlabel('Grip Level (open/soft/med/hard/close)')
axs[1][1].set_ylabel('Grip Width (millimeters)')
axs[1][1].grid(True)

plt.show()