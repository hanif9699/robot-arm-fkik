from threading import Thread
from TrajectoyInterpolator import *
import datetime
from time import sleep
import numpy as np

class RobotExecutionThread(Thread):
    """description of class"""
    def __init__(self, currentConfiguration,  robotPhysics,sampleTrajectoryGenerator, samplingSec):
        Thread.__init__(self)
        self.Run = True
        self.knotPoints = []
        self.robotPhysics = robotPhysics
        self.currentConfiguration = currentConfiguration
        self.sampleTrajectory = sampleTrajectoryGenerator
        self.delNextPointTime = samplingSec
        self.isMoving = False
	self.SuccessExecutedMove= True
      
    def isMovingBlock(self):
        while(self.isMoving):
            sleep(0.1)
	if(self.SuccessExecutedMove==False):
           self.SuccessExecutedMove = True
           return False
        else:
	    return True

    def addKnotPoint(self, knotPoint, isJointSpace, relativeTime = 0):
        self.knotPoints.append([knotPoint, isJointSpace, relativeTime, datetime.datetime.now() ])

    def run(self):
        print("[Info :] Robot Execution Thread started")
        while(self.Run):
            if(len(self.knotPoints) > 0):
		self.isMoving = True
                knotPoint , isJointSpace, relativeTime, starTime = self.knotPoints.pop(0)
                print("[Info :] Executing the robot to this knot point : " + str(knotPoint))
                if(isJointSpace == False):
                    knotPoint = self.robotPhysics.computeInverseKinematics(knotPoint)
		    if(knotPoint is None):
			print("[Error :] Unable to find joint angles Skipping this knot point")
			self.SuccessExecutedMove= False
			continue
                    print("[Info :]The knot point in joint space was found as :" + str(knotPoint))

                if(relativeTime<=0):
                   relativeTime = 1.5*self.robotPhysics.minRelTimeNeeded(np.array(knotPoint) - np.array(self.currentConfiguration))
                   if(relativeTime <= 0):
                        relativeTime = 1.0
                   print("Time of Execution " + str(relativeTime))
                starTime = datetime.datetime.now()
                if(type(self.sampleTrajectory) is TrajectoyCubicInterpolator):
                    traj = TrajectoyCubicInterpolator(self.currentConfiguration, knotPoint, [0] * len(knotPoint) , [0] *len(knotPoint), starTime, relativeTime)
                else:
                    traj = TrajectoyInterpolator(self.currentConfiguration, knotPoint, [0] * len(knotPoint) , [0] *len(knotPoint), starTime, relativeTime)
                while True:
                    t = (datetime.datetime.now() - starTime).total_seconds()
                    if(t>=0 and t <= relativeTime):
                        self.robotPhysics.motorControlFunction(traj.getJointConfigurationToExecute())
                        self.currentConfiguration = traj.getJointConfigurationToExecute()
                        self.isMoving = True
                    else:
                        self.robotPhysics.motorControlFunction(knotPoint)
                        self.currentConfiguration = knotPoint
                        if(len(self.knotPoints) == 0):
                              self.isMoving = False
                        break
                    sleep(self.delNextPointTime)
	    else:
	       self.isMoving = False
            sleep(0.1)

    def __del__(self):
          self.Run =False
          self.join()

                       





