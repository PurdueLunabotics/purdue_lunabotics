# all meters(etc.) unless otherwise mentioned
from math import cos, pi, sin

# https://robotics.jpl.nasa.gov/media/documents/advanced_robotics_2006.pdf
class SlipController:
    waypoints = [(0, 0), (10, 30), (20, 40), (30, 30), (40, 60), (20, 80), (50, 80)]
    MAX_VELOCITY = 13
    MAX_ACCELERATION = 0.7
    pose = {'x':0, 'y':0,'theta':0,'vx':0,'vy':0,'vtheta':0}
    wheelSpeed = [0,0]
    
    def __init__(self):
        pass

    def calculate(self, pose, ):
        wheelSpeeds = []
        
        
        return wheelSpeeds
    
        
        
# Use this to find the theoretical pose (ignores slip)
class WheelOdometry:
    pose = {'x':0, 'y':0,'theta':0,'vx':0,'vy':0,'vtheta':0}
    wheelRadius = .2 
    driveBase = .75 # distance between center of wheels (left/right) meters
    lastTime = 0 #s
    
    def updatePose(self, newTime, wheelSpeeds):
        """Updates the pose based on the accumulated encoder ticks
        of the two wheels. See https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
        for details.
        """
        leftVelo = wheelSpeeds[0] #rotations/s
        rightVelo = wheelSpeeds[1]
        
        deltaTime = newTime - self.lastTime #s
        
        
        
        leftTravel = leftVelo*deltaTime*2*pi*self.wheelRadius # rotations/s * s * m = m
        rightTravel = rightVelo*deltaTime*2*pi*self.wheelRadius

        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / self.driveBase

        if rightTravel == leftTravel:
            deltaX = leftTravel*cos(self.pose['theta'])
            deltaY = leftTravel*sin(self.pose['theta'])
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.pose['x'] - radius*sin(self.pose['theta'])
            iccY = self.pose['y'] + radius*cos(self.pose['theta'])

            deltaX = cos(deltaTheta)*(self.pose['x'] - iccX) \
                - sin(deltaTheta)*(self.pose['y'] - iccY) \
                + iccX - self.pose['x']

            deltaY = sin(deltaTheta)*(self.pose['x'] - iccX) \
                + cos(deltaTheta)*(self.pose['y'] - iccY) \
                + iccY - self.pose['y']

        self.pose["x"] += deltaX
        self.pose["y"] += deltaY
        self.pose["theta"] = (self.pose['theta'] + deltaTheta) % (2*pi)
        self.pose["vx"] = deltaTravel / deltaTime if deltaTime > 0 else 0.
        self.pose["vy"] = 0
        self.pose["vtheta"] = deltaTheta / deltaTime if deltaTime > 0 else 0.

        self.lastTime = newTime

    def getPose(self):
        return self.pose

    def setPose(self, newPose):
        self.pose = newPose
        

def main():
    pass