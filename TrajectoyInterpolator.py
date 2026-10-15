import datetime
import numpy as np


class TrajectoyInterpolator(object):
    """description of class"""

    def __init__(self, qstart, qend, qdotStart, qdotEnd, startTime, deltaT):
        self.qstart = np.array(qstart)
        self.qend = np.array(qend)
        self.qdotStart = np.array(qdotStart)
        self.qdotEnd = np.array(qdotEnd)
        self.deltaT = deltaT
        self.StartTime = np.array(startTime)
        

    def getJointConfigurationToExecute(self):
        delt = (datetime.datetime.now() - self.StartTime).total_seconds()
        if(delt < 0): return self.qstart
        if(delt > self.deltaT): return self.qend
        return self.qstart + (self.qend -self.qstart) * delt /self.deltaT


class TrajectoyCubicInterpolator(TrajectoyInterpolator):

     def __init__(self, qstart, qend, qdotStart, qdotEnd, startTime, deltaT):
         TrajectoyInterpolator.__init__(self, qstart, qend, qdotStart, qdotEnd, startTime, deltaT)
         self.a0 =  self.qstart
         self.a1 = self.qdotStart
         tf = self.deltaT
         self.a2 = (self.qend - self.qstart)*3/(tf*tf) - self.qdotStart*2/(tf) - self.qdotEnd/tf
         self.a3 = (self.qend - self.qstart)*(-2/(tf*tf*tf)) + (self.qdotEnd + self.qdotStart)/(tf*tf)

     def getJointConfigurationToExecute(self):
        t = (datetime.datetime.now() - self.StartTime).total_seconds()
        if(t < 0): return self.qstart
        if(t > self.deltaT): return self.qend
        return self.a0 + self.a1 *t + self.a2 * t*t  + self.a3 * t*t*t
        






