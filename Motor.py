


class Motor(object):
    """description of class"""

    def __init__(self,Name):
        self.Name = Name

    @staticmethod
    def Map(value, value_min,  value_max,  other_min,  other_max):
        ratio = (value - value_min) / (value_max - value_min)
        return other_min + ratio * (other_max - other_min)
        


class Servo(Motor):

    def __init__(self, Name, ChannelNo, PulseDurationMilliSeconds, minPulseDurationMilliSeconds, maxPulseDurationMilliSeconds, minAngleDegrees, maxAngleDegrees):
        Motor.__init__(self, Name)
        self.pwmFrequency = int(1000.0/PulseDurationMilliSeconds)
        self.minAngleDegree = minAngleDegrees
        self.MaxAngleDegree = maxAngleDegrees
        self.MinPulseDurationMS = minPulseDurationMilliSeconds
        self.MaxPulseDurationMS = maxPulseDurationMilliSeconds
        self.ChannelNo = ChannelNo

    

    def getPulseDurationMilliSeconds(self, angle):
        return  Motor.Map(angle, self.minAngleDegree, self.MaxAngleDegree, self.MinPulseDurationMS , self.MaxPulseDurationMS)

    @staticmethod
    def CreateMG995(Name, ChannelNo):
        return Servo(Name, ChannelNo, 20, 0.5,2.5, -90, 90)

    @staticmethod
    def CreateV0150(Name, ChannelNo):
        return Servo(Name, ChannelNo, 20, 0.5,2.5, -90, 90)