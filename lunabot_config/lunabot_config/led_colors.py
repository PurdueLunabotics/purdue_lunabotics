from enum import Enum

class LedColor(Enum):
    OFF = 0
    RED = 1
    ORANGE = 2
    YELLOW = 3
    GREEN = 4
    TEAL = 5
    BLUE = 6
    MAGENTA = 7
    WHITE = 8
    RAINBOW = 9

def colorsToInteger(colors: tuple[LedColor] | LedColor):
    # pack color sequence into an integer. Each digit is a color, 10 possible options

    # if one single color, return its value
    if isinstance(colors, LedColor):
        return colors.value

    if (len(colors) > 5):
        print("Led Colors: Sequences of length greater than 5 are not supported")
        return 0
    
    packedInt = 0
    digitMult = 1
    for color in colors:
        packedInt += color.value * digitMult
        digitMult *= 10

    return packedInt
        