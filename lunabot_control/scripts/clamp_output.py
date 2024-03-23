

"""
Function to clamp output to integer between [-127, 127]
"""
def clamp_output(input):
    return min(max(int(input), -127), 127)