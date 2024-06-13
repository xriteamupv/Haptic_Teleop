def map_range(value, from_min, from_max, to_min, to_max):
    """
    Map a value from one range to another.
    
    :param value: The input value to be mapped.
    :param from_min: The minimum value of the input range.
    :param from_max: The maximum value of the input range.
    :param to_min: The minimum value of the output range.
    :param to_max: The maximum value of the output range.
    :return: The mapped value in the output range.
    """
    # Ensure the input value is within the source range
    value = max(from_min, min(value, from_max))
    
    # Perform the linear interpolation
    mapped_value = to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)
    
    return mapped_value

# Define the input ranges and output range
width_min = 0
width_max = 99
force_min = 0
force_max = 40
vibration_min = 10
vibration_max = 100

def calculate_vibration_intensity(width, force):
    """
    Calculate the vibration intensity based on the width and force values.
    
    :param width: The width variation (0 to 99 mm).
    :param force: The force applied (0 to 40 N).
    :return: The calculated vibration intensity (50 to 100).
    """
    # Map the width and force to intermediate values
    width_mapped = map_range(width, width_min, width_max, vibration_min, vibration_max)
    force_mapped = map_range(force, force_min, force_max, vibration_min, vibration_max)
    
    # Combine the two mapped values to determine the final vibration intensity
    # Here, we can use a simple average of the two values as an example
    vibration_intensity = (width_mapped + force_mapped) / 2
    
    return vibration_intensity

# Example usage
width = 0  # Example width value
force = 20  # Example force value
vibration_intensity = calculate_vibration_intensity(width, force)

print(f"Vibration Intensity: {vibration_intensity}")
