

class RobotPhysics(object):
    """description of class"""

    def __init__(self, motorControlFunction, forwardKinematicsFn, inverKinematicsFn, minVelocityJointSpace, maxVelocityJointSpace):
        self.motorControlFunction = motorControlFunction
        self.computeForwardKinematics = forwardKinematicsFn
        self.computeInverseKinematics = inverKinematicsFn
        self.minVelocityJointSpace = minVelocityJointSpace
        self.maxVelocityJointSpace = maxVelocityJointSpace

    def minRelTimeNeeded(self, dist):
        delT = []
        c = 0
        for x in dist:
            t= 0
            if(x < 0):
                t = x / self.minVelocityJointSpace[c]
            else:
                t = x /self.maxVelocityJointSpace[c]
            delT.append(t)
            c = c + 1
        return max(delT)






        


