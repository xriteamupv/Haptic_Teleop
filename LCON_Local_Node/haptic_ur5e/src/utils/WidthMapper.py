#!/usr/bin/env python3
import numpy as np
from numpy.polynomial.polynomial import Polynomial
from scipy.interpolate import CubicSpline
from sklearn.svm import SVR

class WidthMapper:

    max_wth_var = 99
    max_grip_lvl = 4
    breakpts = [0, 1, 2, 3, 4]
    wdths = [0, 20, 50, 80, 99]

    coefficients = np.polyfit(np.array(breakpts), np.array(wdths), 3)
    poly_default = Polynomial(coefficients[::-1])

    svm_default = SVR(kernel='rbf', C=100, gamma=0.1, epsilon=0.1)
    spline_width = CubicSpline(np.array(breakpts), np.array(wdths), bc_type='natural')

    # ==========================================
    # MAPPING: OBJECTIVE WIDTH vs GRIP LEVEL
    # ==========================================

    @staticmethod
    def linear_interpolation_width(grip_level, max_width_var=max_wth_var, max_grip_level=max_grip_lvl):
        # Map the grip level (0-4) to the objective width (0-99)
        # Using linear interpolation formula: y = ((y2 - y1) / (x2 - x1)) * (x - x1) + y1
        # Here, x1 = 0, x2 = max_grip_level, y1 = 0, y2 = max_width_var
        objective_width = (max_width_var / max_grip_level) * grip_level
        return int(objective_width)

    @staticmethod
    def interval_detection_width(grip_level, breakpoints=breakpts, widths=wdths):
        
        for i in range(1, len(breakpoints)):
            if grip_level <= breakpoints[i]:
                # Linear interpolation within the interval
                x1, x2 = breakpoints[i - 1], breakpoints[i]
                y1, y2 = widths[i - 1], widths[i]
                width = ((y2 - y1) / (x2 - x1)) * (grip_level - x1) + y1
                return int(width)
        
        # Fallback in case the above logic doesn't work as expected
        return widths[-1]
    
    @staticmethod
    def prep_poly_regression_width(levels=breakpts, widths=wdths, degree=3):
        grip_lvls = np.array(levels)
        obj_widths = np.array(widths)
        # Fit a polynomial regression model
        coefficients = np.polyfit(grip_lvls, obj_widths, degree)
        # Polynomial object using the fitted coefficients
        polynomial = Polynomial(coefficients[::-1])
        return polynomial

    @staticmethod
    def polynomial_regression_width(grip_level, polynomial=poly_default, widths=wdths):
        objective_width = polynomial(grip_level)
        objective_width = max(0, min(max(widths), objective_width))
        return int(objective_width)
    
    @staticmethod
    def prep_cubic_splin(levels=breakpts, widths=wdths):
        grip_lvls = np.array(levels)
        obj_widths = np.array(widths)
        spline_width = CubicSpline(grip_lvls, obj_widths, bc_type='natural')
        return spline_width

    @staticmethod
    def cubic_spline_width(grip_level, spline=spline_width, widths=wdths):
        objective_width = spline(grip_level)
        objective_width = max(0, min(max(widths), objective_width))     
        return int(objective_width)
    
    @staticmethod
    def train_support_vector_machine_width(levels=breakpts, widths=wdths):
        grip_levels = np.array(levels).reshape(-1, 1)
        objective_widths = np.array(widths)
        WidthMapper.svm_default.fit(grip_levels, objective_widths)
        return WidthMapper.svr_default

    @staticmethod
    def predict_support_vector_machine_width(grip_level, svm=svm_default):
        objective_width = svm.predict(np.array([[grip_level]]))[0]
        objective_width = max(0, min(99, objective_width))
        return int(objective_width)