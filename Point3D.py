from Vector3D import Vector3D


class Point3D(object):

    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z

    def __repr__(self, **kwargs):
        return str(self.X ) + "," + str(self.Y) + "," + str(self.Z)

    @staticmethod
    def Subtract(Left, Right):
        return Vector3D(Left.X - Right.X, Left.Y - Right.Y, Left.Z - Right.Z)

   
