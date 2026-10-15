from math import * 

class Vector(object):
    
    radToDeg = 180.0 /pi

    def __init__(self, dimension):
        self.dimension = dimension
        self.data = [0] * dimension
        self.initialise()

    
    def setData(self, data):
        for i in range(0, len(data)):
            self.data[i] = data[i]
        self.initialise()
   
    @property
    def X(self):
       return self.data[0]

    @property
    def Y(self):
       return self.data[1]

    @property
    def Z(self):
       return self.data[2]

    @property
    def X(self, Value):
       self.data[0] = Value

    @property
    def Z(self, Value):
       self.data[1] = Value

    @property
    def Z(self, Value):
       self.data[2] = Value

    @staticmethod
    def Create(noDimension):
        return Vector(noDimension)

    @staticmethod
    def Create(strData, delimiter):
        floats = [float(x) for x in strData.split(delimiter)]
        v = Vector(len(floats))
        v.setData(floats)


    def compute(val):
        return isnan(val)

    def  initialise(self):
        self.lengthSq = float('nan');
        self.length = float('nan');
        self.angles = [None] * self.dimension;
        self.min = float('nan');
        

    def LengthSq(self):
        if (compute(self.lengthSq)):
            self.lengthSq = 0
            for i in range(0, self.dimension):
                    self.lengthSq = self.lengthSq + self.data[i] * self.data[i]

        return self.lengthSq;
    
    def Length(self):
           if (compute(length)):
               self.LengthSq()
               self.length = sqrt(lengthSq)
            
           return length;
        
    def  Normalize(self):
            self.Length();

            if (self.length != 1):
                for i in range(0, self.dimension):
                    self.data[i] =  self.data[i] / self.length;

            initialise();
            self.length = 1;
            self.LengthSq = 1;

    def Dot(self, Right):
        res = 0
        for i in range(0, self.dimension):
            res = res + self.data[i] * Right.data[i];

        return res;

    def directionCosines(self):
            if (self.angles[0] == None):
                self.Length();
                for i in range(0, self.dimension):
                    self.angles[i] = acos(data[i] / length) * radToDeg
            
            return angles;


    def Scale(self, m):
        for i in range(0, self.dimension):
                self.data[i] *= m;

        initialise();

    @staticmethod
    def IsWithinBounds(self, minPt, maxPt):
            #ptWithinBound = new Vector(dimension);
            outside = False
            for i in range(0, self.dimension):
                if (self.data[i] < self.minPt.data[i]):
                    #ptWithinBound.data[i] = minPt.data[i];
                    outside = True
                else:
                    if (self.data[i] > maxPt.data[i]):
                    #ptWithinBound.data[i] = maxPt.data[i];
                        outside = True
                
            return outside == False

    def minValueInVector(self):
            if (self.compute(self.min)):
                self.min = min( self.data)

            return self.min

    def __repr__(self):
         res = ""
         for i in range(0, self.dimension):
                res +=  data[i].ToString(format) + " ";

         return res;

    def  Clone(self):
        return copy.deepcopy(self)

    def __eq__(self, other):
         for i in range(0, self.dimension):
             if (self.data[i] != other.data[i]):
                  return False
         return True
        
    def EuclideanDistanceSq(self, other):
            value = 0;
            dim = min([self.dimension, other.dimension])
            for i in range(0, dim):
                v = self.data[i] - other.data[i]
                value += v * v;
            
            return value
        
    def __add__(self, Right):
            v = Vector(self.dimension)

            for i in range(0, self.dimension):
                v.data[i] = self.data[i] + Right.data[i];

            return v;

    def __sub__(self, Right):
            v = Vector(self.dimension)

            for i in range(0, self.dimension):
                v.data[i] = self.data[i] - Right.data[i];

            return v;
        
    def __mul__(self, constant):
            v = Vector(self.dimension)

            for i in range(0, self.dimension):
                v.data[i] = self.data[i] * constant;

            return v;


    @staticmethod
    def AngleBetween( va1, va2):
            v1 = va1.Clone()
            v2 = va2.Clone()

            v1.Normalize()
            v2.Normalize()

            if (v1 == v2):
                return 0
            

            return acos(min([1.0, v1.Dot(v2)])) * radToDeg;

    @staticmethod
    def  Cross(Left, Right):
            v = Vector(3)
            data = [Left.Y * Right.Z - Left.Z * Right.Y, Left.Z * Right.X - Left.X * Right.Z, Left.X * Right.Y - Left.Y * Right.X]
            v.setData(data)
            return v
            




