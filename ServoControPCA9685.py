from Motor import Servo
from math import *

class ServoControl(object):
    """description of class"""

    def __init__(self, ServoMotorArrangement, minJointAngles, maxJointAngles):
        self.ServoArrangement = ServoMotorArrangement
        self.NoOfJoints = len(ServoMotorArrangement)
        self.minJointAngles = minJointAngles
        self.maxJointAngles = maxJointAngles

    def setAngles(self,angles):
        #print("[Info :] Setting joint angles to " + str(angles))
        for i in range(0, self.NoOfJoints):
            if(angles[i] < self.minJointAngles[i] or angles[i] > self.maxJointAngles[i]):
                #print("[Warning :]  Joint angle " + str(i) + "out of bounds")
                continue
            for x in self.ServoArrangement[i]:
                pulseDuration = x.getPulseDurationMilliSeconds(angles[i])
                self.SendCommandPulse(x.ChannelNo, pulseDuration)

    def SendCommandPulse(self, ChNO, pulseDuration ):
        x = 0
        #print("[info :] Pulse Duration for ChNo : " + str(ChNO) + " set is " + str(pulseDuration) +"ms")
 

import Adafruit_PCA9685

class ServoControlPCA9685(ServoControl):

    def __init__(self, ServoMotorArrangement,minJointAngles, maxJointAngles, SCLSDAAddress = 0x40):
        ServoControl.__init__(self,ServoMotorArrangement, minJointAngles, maxJointAngles)
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685(SCLSDAAddress)
        # Set frequency to 
        self.FrequencyOperation = ServoMotorArrangement[0][0].pwmFrequency
        self.pwm.set_pwm_freq(self.FrequencyOperation)
       
        

    def SendCommandPulse(self, ChNO, pulseDuration):
        super(ServoControlPCA9685, self).SendCommandPulse(ChNO, pulseDuration)
        offAt = ceil((pulseDuration *self.FrequencyOperation/1000.0) * 4096)
        if(ChNO==0):
             print (offAt)
        self.pwm.set_pwm(ChNO, 0, int(offAt))


 
if __name__ == '__main__':
    ServoArrangement = [[Servo.CreateV0150("Base", 0)],
                        [Servo.CreateMG995("ShoulderLeft", 1), Servo.CreateMG995("ShoulderRight", 2)]
                        ]
    servoControl = ServoControlPCA9685(ServoArrangement)
    servoControl.setAngles([10, 25])

