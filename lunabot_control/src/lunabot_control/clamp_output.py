
def clamp_output(input):
    """
    Function to clamp output to integer between [-127, 127]
    """
    return min(max(int(input), -127), 127)