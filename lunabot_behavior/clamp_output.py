
def clamp_output(input): # TODO RJN - set to RPM now
    """
    Function to clamp output to integer between [-127, 127]
    """
    return min(max(int(input), -127), 127)