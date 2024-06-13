#!/usr/bin/env python3

class HapticMapper:

    @staticmethod
    def map_range(value, base_domain, map_domain):
        value = max(base_domain[0], min(value, base_domain[1]))
        mapped_value = map_domain[0] + (value - base_domain[0]) * (map_domain[1] - map_domain[0]) / (base_domain[1] - base_domain[0])
        return mapped_value

    @staticmethod
    def linear_interpolation(width_var, force, limitator):
        """
        Calculate the vibration intensity based on linear interpolations of the width and force values.
        """
        # Map the width and force to intermediate values
        width_mapped = HapticMapper.map_range(width_var, limitator.grip_width_limits, limitator.intensity_limits)
        force_mapped = HapticMapper.map_range(force, limitator.grip_force_limits, limitator.intensity_limits)
        
        # Combine the two mapped values to determine the final vibration intensity
        # Here, we can use a simple average of the two values as an example
        vibration_intensity = (width_mapped + force_mapped) / 2
        
        return vibration_intensity
    
    def non_linear_transform(value, factor):
        """
        Apply non-linear transformation to the value.
        """
        return (value ** factor)

    @staticmethod
    def weighted_interpolation(width_var, force, limitator, non_linear_scaling_factor=1.5):
        """
        Calculate the vibration intensity based on the width and force values using a weighted combination.
        """
        # Map the width and force to intermediate values
        width_mapped = HapticMapper.map_range(width_var, limitator.grip_width_limits, limitator.intensity_limits)
        force_mapped = HapticMapper.map_range(force, limitator.grip_force_limits, limitator.intensity_limits)
        
        # Apply non-linear scaling
        width_mapped = HapticMapper.non_linear_transform(width_mapped - limitator.intensity_limits[0], non_linear_scaling_factor) + limitator.intensity_limits[0]
        force_mapped = HapticMapper.non_linear_transform(force_mapped - limitator.intensity_limits[0], non_linear_scaling_factor) + limitator.intensity_limits[0]
        
        # Combine the two mapped values using the weights
        vibration_intensity = limitator.weights[0] * width_mapped + limitator.weights[1] * force_mapped
        
        # Ensure the vibration intensity is within the output range
        vibration_intensity = max(limitator.intensity_limits[0], min(vibration_intensity, limitator.intensity_limits[1]))
        
        return vibration_intensity
    
    def dynamic_weights(width, force, thresholds):
        """
        Adjust weights dynamically based on thresholds.
        """
        if width > thresholds[0]:
            weight_width = 0.8
            weight_force = 0.2
        elif force > thresholds[1]:
            weight_width = 0.4
            weight_force = 0.6
        else:
            weight_width = 0.6
            weight_force = 0.4
            
        return weight_width, weight_force

    def dynamically_weighted_interpolation(width_var, force, limitator, non_linear_scaling_factor=1.5):
        """
        Calculate the vibration intensity based on the width and force values.
        """
        # Map the width and force to intermediate values
        width_mapped = HapticMapper.map_range(width_var, limitator.grip_width_limits, limitator.intensity_limits)
        force_mapped = HapticMapper.map_range(force, limitator.grip_force_limits, limitator.intensity_limits)
        
        # Apply non-linear scaling
        width_transformed = HapticMapper.non_linear_transform(width_mapped, non_linear_scaling_factor)
        force_transformed = HapticMapper.non_linear_transform(force_mapped, non_linear_scaling_factor)
        
        # Adjust weights dynamically
        weight_width, weight_force = HapticMapper.dynamic_weights(width_var, force, limitator.thresholds)
        
        # Combine the two mapped values using the dynamic weights
        vibration_intensity = weight_width * width_transformed + weight_force * force_transformed
        
        # Ensure the vibration intensity is within the output range
        vibration_intensity = max(limitator.intensity_limits[0], min(vibration_intensity, limitator.intensity_limits[1]))
        
        return vibration_intensity
    
    def biased_dynamic_weights(width, force, bias=[0.5, 0.5]):
        """
        Adjust weights dynamically based on biases.
        """
        # Normalize biases to ensure their sum is 1
        total_bias = bias[0] + bias[1]
        weight_width = bias[0] / total_bias
        weight_force = bias[1] / total_bias
        
        return [weight_width, weight_force]

    def normalize(value, min_value, max_value):
        """
        Normalize the value to be within the specified range.
        """
        return (value - min_value) / (max_value - min_value)

    def normalized_biased_weighted_interpolation(width_var, force, limitator,
                                    width_exponent=1.5, force_exponent=1.5, 
                                    bias_width=0.5, bias_force=0.5,
                                    vibration_min=50, vibration_max=100):
        """
        Calculate the vibration intensity based on the width and force values using a normalized biased approach.
        """
        # Map the width and force to intermediate values
        width_mapped = HapticMapper.map_range(width_var, limitator.grip_width_limits, limitator.intensity_limits)
        force_mapped = HapticMapper.map_range(force, limitator.grip_force_limits, limitator.intensity_limits)
        
        # Apply exponential scaling
        width_scaled = HapticMapper.non_linear_transform(width_mapped, width_exponent) # exponential
        force_scaled = HapticMapper.non_linear_transform(force_mapped, force_exponent)
        
        # Normalize the scaled values
        width_normalized = HapticMapper.normalize(width_scaled, vibration_min ** width_exponent, vibration_max ** width_exponent)
        force_normalized = HapticMapper.normalize(force_scaled, vibration_min ** force_exponent, vibration_max ** force_exponent)
        
        # Adjust weights dynamically
        weight_width, weight_force = HapticMapper.dynamic_weights(width_var, force, bias_width, bias_force)
        
        # Combine the normalized values using the dynamic weights
        vibration_intensity = (weight_width * width_normalized + weight_force * force_normalized) * (vibration_max - vibration_min) + vibration_min
        
        # Ensure the vibration intensity is within the output range
        vibration_intensity = max(vibration_min, min(vibration_intensity, vibration_max))
        
        return vibration_intensity