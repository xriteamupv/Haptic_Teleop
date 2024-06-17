import math
import numpy as np
from scipy.interpolate import CubicSpline
from sklearn.svm import SVR

class ForceMapper:

    dur_limits = [1000, 10000]
    f_limits = [0, 70]

    # ==========================================
    # MAPPING: OBJECTIVE FORCE vs TIME DURATION
    # ==========================================

    @staticmethod
    def linear_mapping_force(duration, duration_limits=dur_limits, force_limits=f_limits):
    
        if duration < duration_limits[0]:
            duration = duration_limits[0]
        elif duration > duration_limits[1]:
            duration = duration_limits[1]

        # Linear mapping from duration to force
        force = force_limits[0] + (force_limits[1] - force_limits[0]) * (duration_limits[1] - duration) / (duration_limits[1] - duration_limits[0])
        
        return force
    
    @staticmethod
    def exponential_decay_force(duration, duration_limits=dur_limits, force_limits=f_limits, decay_rate=5):
        
        duration = max(duration_limits[0], min(duration_limits[1], duration))
        normalized_duration = (duration - duration_limits[0]) / (duration_limits[1] - duration_limits[0])
        force = force_limits[0] + (force_limits[1] - force_limits[0]) * math.exp(-decay_rate * normalized_duration)
        
        return force
    
    @staticmethod
    def piecewise_quadratic_mapping_force(duration, duration_limits=dur_limits, force_limits=f_limits):
        
        breakpoints = [duration_limits[0], (duration_limits[0] + duration_limits[1]) / 2, duration_limits[1]]
        duration = max(duration_limits[0], min(duration_limits[1], duration))
        
        # Normalized duration within each segment
        if duration <= breakpoints[1]:
            # First segment: Quadratic decrease
            normalized_duration = (duration - duration_limits[0]) / (breakpoints[1] - duration_limits[0])
            force = force_limits[1] - (force_limits[1] - (force_limits[1] + force_limits[0]) / 2) * normalized_duration**2
        else:
            # Second segment: Quadratic decrease
            normalized_duration = (duration - breakpoints[1]) / (duration_limits[1] - breakpoints[1])
            force = (force_limits[1] + force_limits[0]) / 2 - ((force_limits[1] + force_limits[0]) / 2 - force_limits[0]) * normalized_duration**2
        
        return force
    
    @staticmethod
    def cubic_spline_force(duration, duration_limits=dur_limits, force_limits=f_limits):
        
        duration_points = np.array([duration_limits[0], (duration_limits[0] + duration_limits[1]) / 3, (duration_limits[0] + duration_limits[1]) * 2 / 3, duration_limits[1]])
        force_points = np.array([force_limits[1], force_limits[1] / 2, force_limits[1] / 4, force_limits[0]])
        
        spline = CubicSpline(duration_points, force_points)
        duration = max(duration_limits[0], min(duration_limits[1], duration))
        force = spline(duration)
        
        return force