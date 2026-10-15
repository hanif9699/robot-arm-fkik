from math import *
import numpy as np


class ImageIJToXY(object):
    """description of class"""

    def __init__(self, pmin, pmax, Pmin, Pmax):
        self.pmin = pmin
        self.pmax = pmax
        self.Pmin = Pmin
        self.Pmax = Pmax

        self.center = (self.Pmax + self.Pmin)*0.5

        self.distReal = sqrt(np.sum( (self.Pmax - self.Pmin)*(self.Pmax - self.Pmin)))
        self.distPixels = sqrt(np.sum( (self.pmax - self.pmin)*(self.pmax - self.pmin)))

        self.ExchangeXY = True
        self.FlipX = True
        self.FlipY = True

    def map(self, value, value_min, value_max, other_min, other_max ):
        ratio = (float(value) - float(value_min)) / (float(value_max) - float(value_min))
        return float(other_min) + ratio * (float(other_max) - float(other_min))

    def pixelDistance(self, distance):
        ratio = float(distance) / float(self.distReal)
        return floor(ratio * self.distPixels)

    def physicalDistance(self, distance):
         ratio = float(distance) / float(self.distPixels)
         return ratio * self.distReal

    def Flip(self, center, value):
        return -(value - center) + center

    def getPose(self, i, j):
        if(self.ExchangeXY):
            X = self.map(j, self.pmin[1], self.pmax[1], self.Pmin[0], self.Pmax[0])
            Y = self.map(i, self.pmin[0], self.pmax[0], self.Pmin[1], self.Pmax[1])
            if(self.FlipX):
                X = self.Flip(self.center[0], X)
            if(self.FlipY):
                Y = self.Flip(self.center[1], Y)
            return X, Y
        else:
            X = self.map(i, self.pmin[0], self.pmax[0], self.Pmin[0], self.Pmax[0])
            Y = self.map(j, self.pmin[1], self.pmax[1], self.Pmin[1], self.Pmax[1])
            if(self.FlipX):
                X = self.Flip(self.center[0], X)
            if(self.FlipY):
                Y = self.Flip(self.center[1], Y)
            return X, Y

    def getPixel(self, X, Y):
        if(self.ExchangeXY):
            if(self.FlipX):
                X = self.Flip(self.center[0], X)
            if(self.FlipY):
                Y = self.Flip(self.center[1], Y)
            i = self.map(Y, self.Pmin[1], self.Pmax[1], self.pmin[0], self.pmax[0])
            j = self.map(X, self.Pmin[0], self.Pmax[0], self.pmin[1], self.pmax[1])
            return i,  j
        else:
            if(self.FlipX):
                X = self.Flip(self.center[0], X)
            if(self.FlipY):
                Y = self.Flip(self.center[1], Y)
            i = self.map(X, self.Pmin[0], self.Pmax[0], self.pmin[0], self.pmax[0])
            j = self.map(Y, self.Pmin[1], self.Pmax[1], self.pmin[1], self.pmax[1])





