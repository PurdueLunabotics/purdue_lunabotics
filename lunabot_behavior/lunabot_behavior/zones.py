from geometry_msgs.msg import Point
from lunabot_msgs.msg import Zone

# zone geometries - based on guidebook orientations (all measurements in meters)
START_OFFSET_X = 2.44
START_OFFSET_Y = 1.5
START_LENGTH_X = 2
START_LENGTH_Y = 2

EXC_OFFSET_X = 2.19
EXC_OFFSET_Y = 0
EXC_LENGTH_X = 2.5
EXC_LENGTH_Y = 5

BERM_OFFSET_X = -1.94
BERM_OFFSET_Y = 1.9
BERM_LENGTH_X = 1.7
BERM_LENGTH_Y = 0.8

def make_zone(offset_x, offset_y, length_x, length_y):
    z = Zone()

    z.v1 = Point()
    z.v1.x = offset_x + (length_x / 2)
    z.v1.y = offset_y + (length_y / 2)

    z.v2 = Point()
    z.v2.x = offset_x + (length_x / 2)
    z.v2.y = offset_y - (length_y / 2)

    z.v3 = Point()
    z.v3.x = offset_x - (length_x / 2)
    z.v3.y = offset_y - (length_y / 2)

    z.v4 = Point()
    z.v4.x = offset_x - (length_x / 2)
    z.v4.y = offset_y + (length_y / 2)

    return z

start_zone = make_zone(
    START_OFFSET_X,
    START_OFFSET_Y,
    START_LENGTH_X,
    START_LENGTH_Y)

exc_zone = make_zone(
    EXC_OFFSET_X,
    EXC_OFFSET_Y,
    EXC_LENGTH_X,
    EXC_LENGTH_Y)

berm_zone = make_zone(
    BERM_OFFSET_X,
    BERM_OFFSET_Y,
    BERM_LENGTH_X,
    BERM_LENGTH_Y)
