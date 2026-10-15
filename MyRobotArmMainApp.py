from __future__ import print_function
from ServoControPCA9685  import ServoControlPCA9685
import RobotArmFKIK as rk
from TrajectoyInterpolator import *
from TransformationMatrix import TransformationMatrix
import numpy as np
from time import sleep
import RobotExecutionThread
import RobotPhysics
from Motor import Servo
import ImageProcessingLogic
import ImageIJToXY



class MyRobotArmMainApp(object):
    """description of class"""
    servoControl1 = None
    servoControl2 = None
    r = None
    DOF = 5
    threadRun = None
    MinJointAngles = None
    MaxJointAngles = None
    MinGripperJointAngle = 0.0
    MaxGripperJointAngle = 90.0

    def __init__(self):
        ServoArrangement1 = [[Servo.CreateV0150("Base", 1)],
                             #[Servo.CreateV0150("ShoulderLeft", 2), Servo.CreateV0150("ShoulderRight", 3)],
                             [Servo.CreateMG995("ElbowLeft1", 4), Servo.CreateMG995("ElbowRight1", 5)],
                        [Servo.CreateMG995("ElbowLeft2", 6), Servo.CreateMG995("ElbowRight2", 7)],
                        [Servo.CreateMG995("GripperRoll", 8)]
                        ]
        minJointAngles = [ -90.0, -50.0 ,-90.0, -90.0, -90.0]
        
        maxJointAngles = [90.0, 90.0, 90.0 , 90.0, 90.0]
        MyRobotArmMainApp.MinJointAngles = minJointAngles
        MyRobotArmMainApp.MaxJointAngles = maxJointAngles
        MyRobotArmMainApp.servoControl1 = ServoControlPCA9685(ServoArrangement1, [ minJointAngles[0], minJointAngles[2], minJointAngles[3], minJointAngles[4]], [maxJointAngles[0],maxJointAngles[2], maxJointAngles[3], maxJointAngles[4]],0x40)
        ServoArrangement2 = [[Servo.CreateV0150("ShoulderLeft", 14), Servo.CreateV0150("ShoulderRight", 15)],
                             [Servo.CreateMG995("GripperMotor", 9)]
                        ]
        
        MyRobotArmMainApp.servoControl2 = ServoControlPCA9685(ServoArrangement2,[ minJointAngles[1],MyRobotArmMainApp.MinGripperJointAngle ], [maxJointAngles[1], MyRobotArmMainApp.MaxGripperJointAngle], 0x41)
        self.setupDHParameters()
        self.MinVelocity = [-30.0,-30.0,-30.0,-30.0,-30.0]
        self.MaxVelocity = [30.0, 30.0, 30.0, 30.0, 30.0]
        self.StartCfg = [0.0]*MyRobotArmMainApp.DOF
        MyRobotArmMainApp.ExecuteOnHardware(self.StartCfg)
        self.robotPhysics = RobotPhysics.RobotPhysics(MyRobotArmMainApp.ExecuteOnHardware, MyRobotArmMainApp.ForwardKinematics, MyRobotArmMainApp.InverseKinematics,self.MinVelocity, self.MaxVelocity)
        MyRobotArmMainApp.threadRun = RobotExecutionThread.RobotExecutionThread(self.StartCfg, self.robotPhysics, TrajectoyInterpolator([0] * self.DOF, [0] * self.DOF, [0] * self.DOF,[0] * self.DOF,-1, 0.5), 0.01)
        MyRobotArmMainApp.threadRun.start()
    

    def setupDHParameters(self):
        MyRobotArmMainApp.r = rk.Kinematics.RobotArmMechanism()
        L1 = 18.0
        L2= 26.0
        L3 = 6.0 + 14.0 # for gripper
        MyRobotArmMainApp.r.addSegment("None", 0.0, 0.0, 0.0, 0.0)
        MyRobotArmMainApp.r.addSegment("RotZ", 0.0, -90.0, 0.0, 0.0)
        MyRobotArmMainApp.r.addSegment("RotZ", L1, 0.0, 0.0, 0.0)
        MyRobotArmMainApp.r.addSegment("RotZ", L2, 0.0, 0.0, 0.0)
        MyRobotArmMainApp.r.addSegment("RotZ", L3, 90.0, 0.0, 0.0)
        #MyRobotArmMainApp.r.addSegment("RotZ", 0.0, 90.0, 0.0, 0.0)
        MyRobotArmMainApp.r.addSegment("RotZ")
        MyRobotArmMainApp.r.finaliseChain()
        MyRobotArmMainApp.DOF = 5;

    @staticmethod
    def ExecuteOnHardware(angles):
        MyRobotArmMainApp.servoControl1.setAngles([angles[0], angles[2], angles[3], angles[4]])
        MyRobotArmMainApp.servoControl2.setAngles([angles[1], -180.0])

    @staticmethod
    def ForwardKinematics(angles):
        return MyRobotArmMainApp.r.computeForwardKinematics(angles)

    @staticmethod
    def InverseKinematics(Matrix, guessAngles=None):
        angles = [0.0] * MyRobotArmMainApp.DOF
        angles = np.array(angles)
        if(guessAngles is None or len(guessAngles) < MyRobotArmMainApp.DOF):
            if(Matrix[1][3] < 0.0 ):
               guessAngles = [-30.0, -90.0, 90.0, 0.0, 0.0]#MyRobotArmMainApp.threadRun.currentConfiguration
            else:
                guessAngles = [30.0, -90.0, 90.0, 0.0, 0.0]
        #guessAngles = [0.0, 0.0, 0.0, 90.0, 0.0]
        mult = 0
        no = 0
        while(True):
            if(MyRobotArmMainApp.r.computeInverseKinematics(Matrix, guessAngles, angles)):
                 guessAngles = list(angles)
                 angles[1] = angles[1] + 90.0
                 angles[2] = angles[2] - 90.0
                 angles[3] = 90.0-1* angles[3]
                 print(str(no) +": [" + str(Matrix[0][3]) + ", " + str(Matrix[1][3]) + ", "+ str(Matrix[2][3]) + "]-->"+ str(angles), end = '\r')
                 changed = False
                 for i in range(0, 5):
                     if(angles[i] < MyRobotArmMainApp.MinJointAngles[i]-2.0):
                         guessAngles[i] = 45.0
                         changed = True
                         if(i >= 1 and no%2 == 0):
                             guessAngles[i-1] = -1.0 * guessAngles[i-1]
                     if(angles[i] > MyRobotArmMainApp.MaxJointAngles[i]+2.0):
                         guessAngles[i] = -45.0
                         changed = True
                         if(i >= 1 and no%2== 0):
                             guessAngles[i-1] = -1.0 * guessAngles[i-1]
                 if(changed == False):
                      break
                 no = no + 1
                 if(no > 20):
                         if(Matrix[1][3] < 0.0 ):
                             guessAngles = [-30.0, 0.0, 0.0, 0.0, 0.0]#MyRobotArmMainApp.threadRun.currentConfiguration
                         else:
                             guessAngles = [30.0, 0.0, 0.0, 0.0, 0.0]
                         no = 0
                         mult = mult + 1
                         if(mult > 20):
                             return None
                      #sleep(10)
                 #print(guessAngles)
        return angles
            
        

    def closeGripper(self):
        print("[Info :] Closing Gripper")
        MyRobotArmMainApp.servoControl2.setAngles([-180.0, MyRobotArmMainApp.MinGripperJointAngle])

    def openGripper(self):
        print("[Info :] Opening Gripper")
        MyRobotArmMainApp.servoControl2.setAngles([-180.0, MyRobotArmMainApp.MaxGripperJointAngle])

    def setPoseTo(self, X, Y, Z, alpha, beta, gamma, relTimeDuration = 0):
        T = TransformationMatrix.EulerAngles(alpha, beta, gamma)
        T.setPosition(X, Y , Z)
        MyRobotArmMainApp.threadRun.addKnotPoint(T.M, False, relTimeDuration)

    def setJointAnglesTo(self, angles, relTimeDuration = 0):
        MyRobotArmMainApp.threadRun.addKnotPoint(angles, True, relTimeDuration)

    def waitTillExecutionIsComplete(self):
        return MyRobotArmMainApp.threadRun.isMovingBlock()

    def PlaceItInWetBin(self):
        print("[Info:] Placing garbage in wet bin")
        self.setJointAnglesTo([0.0,0.0,0.0,0.0,0.0])
        sleep(3)
        self.waitTillExecutionIsComplete()
        self.setJointAnglesTo([0.0,0.0,-80.0,0.0,0.0])
        self.setJointAnglesTo([55.0,-10.0,-50.0,0.0,0.0])
        self.setJointAnglesTo([55.0,-10.0,-60.0,90.0,0.0])
        sleep(1)
        self.waitTillExecutionIsComplete()
        

    def ReAlign(self):
        self.setJointAnglesTo([0.0, 0.0,-80.0, 0.0,0.0])
        self.setJointAnglesTo([0.0,0.0, -40.0,0.0,0.0])
        self.setJointAnglesTo([0.0,0.0, 0.0,0.0,0.0])
        sleep(1)
        self.waitTillExecutionIsComplete()
        
    def PlaceItInDryBin(self):
        print("[Info:] Placing Garbage in dry bin")
        self.setJointAnglesTo([0.0,0.0,0.0,0.0,0.0])
        sleep(3)
        self.waitTillExecutionIsComplete()
        self.setJointAnglesTo([0.0,0.0,-80.0,0.0,0.0])
        self.setJointAnglesTo([-80.0,-10.0,-50.0,0.0,0.0])
        self.setJointAnglesTo([-80.0,-10.0,-60.0,90.0,0.0])
        sleep(1)
        self.waitTillExecutionIsComplete()


    def __del__(self):
        MyRobotArmMainApp.threadRun.Run =False
        MyRobotArmMainApp.threadRun.join()

def testRobotMotion():
     myRobot = MyRobotArmMainApp()
     sleep(2)
     myRobot.openGripper()
     sleep(3)
     myRobot.closeGripper()
     sleep(1)
     myRobot.PlaceItInDryBin()
     
     #myRobot.setJointAnglesTo([0.0,0.0,-25.0,0.0,0.0])
     #myRobot.setPoseTo(29.4, 15.5, -25.5, 0.0, 90.0, 0.0)
     #myRobot.setJointAnglesTo([48.26203348,-75.1723303 +90.0  , 125.11560078-90.0 , -49.94327048  ,-48.26203348])
     #myRobot.setJointAnglesTo([-30.11373315,  105.80538128-90.0, -137.91344702 +90.0,   32.10806574,   30.11373315])
     #print(MyRobotArmMainApp.ForwardKinematics([10.0,0.0,0.0,0.0,0.0]))
     #myRobot.setJointAnglesTo([45.0,40.0,-40.0,40.0,90.0])
     #myRobot.setJointAnglesTo([0.0,0.0,0.0,0.0,0.0])
     sleep(2)
     myRobot.waitTillExecutionIsComplete()
     sleep(3)
     myRobot.openGripper()
     sleep(2)
     myRobot.ReAlign()
     sleep(2)
     myRobot.waitTillExecutionIsComplete()
     sleep(1)
     del myRobot

def WorkFlow():
    myRobot = MyRobotArmMainApp()
    converter = ImageIJToXY.ImageIJToXY(np.array([0, 0]), np.array([640, 480]), np.array([7.5,-24.5 ]), np.array([42.0, 24.5]))
    ip = ImageProcessingLogic.ImageProcessingLogic(converter)
    #ip.start()  
    Z = -33.0 + 17.0
    

    while True:
      try:
        print("[Info :] Going to home position to take snapshot ")
        myRobot.setJointAnglesTo([0.0,0.0,-15.0,0.0,0.0])
        sleep(1)
        myRobot.waitTillExecutionIsComplete()
        sleep(3)
        while(len(ip.garbagePoses) == 0):
            ip.ProcessFromWebCamera()
            #ip.ShowImage()
            sleep(0.1)
        sleep(1)
        print("[Info :] Found garbage from the snapshot Total = " + str(len(ip.garbagePoses)))
        while(len(ip.garbagePoses) > 0):
            myRobot.openGripper()
            sleep(0.5)
            pose = ip.garbagePoses.pop(0)
            print(str(pose[0]) + ", " + str(pose[1]) + " , " + str(pose[2]))
            #sleep(35)
            myRobot.setPoseTo(pose[0], pose[1], Z, pose[2], 90.0, 0.0)
            sleep(1)
            res = myRobot.waitTillExecutionIsComplete()
            print(res)
            sleep(30)
            if(res == True):
                print("[Info :] Grabbing garbage from the snapshot ")
                myRobot.closeGripper()
                myRobot.PlaceItInWetBin()
                myRobot.ReAlign()
                sleep(5)
        sleep(1)
      except:
          del ip
          del myRobot

if __name__ == '__main__':
    #myRobot = MyRobotArmMainApp()
    #testRobotMotion()
    WorkFlow()



    
