from Vector3D import Vector
from math import *

class Tool:

    @staticmethod     
    def ToRadians( angle_degrees):
                return angle_degrees * pi / 180.0

    @staticmethod
    def ToDegrees( angle_radians):
                return angle_radians * 180 / pi
    
    @staticmethod
    def C(angle_radians):
                return cos(angle_radians)
    @staticmethod
    def S( angle_radians):
                return sin(angle_radians)


class TransformationMatrix(object):
    """description of class"""

    def __init__(self):
        self.M = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]

    def getRotationMatrix(self):
        return [[self.M[0][0], self.M[0][1], self.M[0][2]], [self.M[1][0], self.M[1][1], self.M[1][2]],[self.M[2][0], self.M[2][1], self.M[2][2]]]

    def __str__(self, **kwargs):
        return self.M

    def Multiply(self, Right):
                result = TransformationMatrix();

                for  i in range (0, 4):
                    for j in range (0, 4):
                        sum = 0
                        for k in range (0, 4):
                            sum = sum + self.M[i][k] * Right.M[k][j];
                        result.M[i][j] = sum;
                    
                return result;

    def TransposeRotation(self):
        for  i in range (0, 4):
                    for j in range (0, 4):
                        t = self.M[i][j]
                        self.M[i][j] = self.M[j][i]
                        self.M[j][i] = t

    def Invert(self):
       self.TransposeRotation()

       self. M[0][3] = self.M[0][0] * self.M[0][3] + self.M[0][1] * self.M[1][3] + self.M[0][2] * self.M[2][3]
       self.M[1][3] = self.M[1][0] * self.M[0][3] + self.M[1][1] * self.M[1][3] + self.M[1][2] * self.M[2][3]
       self.M[2][3] = self.M[2][0] * self.M[0][3] + self.M[2][1] * self.M[1][3] + self.M[2][2] * self.M[2][3]

    @staticmethod
    def Create( Path, delimiter):
        res = TransformationMatrix()
        with open(Path) as f:
            lines = f.readlines()
            for  i in range (0, 4):
                 data = lines[i].split(delimiter)
                 for j in range (0, 4):
                     res.M[i][j] = data[j]
        return res

    def ToFile(self, delimiter):
         with open(Path) as f:
              for  i in range (0, 4):
                  line = ""
                  for j in range (0, 3):
                       line = line + str(self.M[i][j]) + delimiter
                  line = line + str(self.M[i][3])
                  f.write(line + "\n")

    def Transform(self, p):
        x = self.M[0][0] * p.X +  self.M[0][1] * p.Y +  self.M[0][2] * p.Z +  self.M[0][3]
        y = self.M[1][0] * p.X +  self.M[1][1] * p.Y +  self.M[1][2] * p.Z +  self.M[1][3]
        z = self.M[2][0] * p.X +  self.M[2][1] * p.Y + self. M[2][2] * p.Z +  self.M[2][3]

    def setPosition(self, X, Y, Z):
	self.M[0][3] = X
        self.M[1][3] = Y
        self.M[2][3] = Z
		
    @staticmethod
    def TranslationMatrix(X, Y, Z):
        m = TransformationMatrix()
        m.M[0][3] = x
        m.M[1][3] = y
        m.M[2][3] = z

        return m

    @staticmethod
    def RotateAboutXAxis(angle_degrees):
        angleRadians = Tool.ToRadians(angle_degrees)
        m = TransformationMatrix()

        m.M[0][0] = 1; m.M[0][1] = 0; m.M[0][2] = 0; m.M[0][3] = 0;
        m.M[1][0] = 0; m.M[1][1] = Tool.C(angleRadians); m.M[1][2] = -Tool.S(angleRadians); m.M[1][3] = 0;
        m.M[2][0] = 0; m.M[2][1] = Tool.S(angleRadians); m.M[2][2] = Tool.C(angleRadians); m.M[2][3] = 0;
        m.M[3][0] = 0; m.M[3][1] = 0; m.M[3][2] = 0; m.M[3][3] = 1;

        return m
    @staticmethod
    def RotateAboutYAxis(angle_degrees):
        angleRadians = Tool.ToRadians(angle_degrees)
        m = TransformationMatrix()
        
        m.M[0][0] = Tool.C(angleRadians); m.M[0][1] = 0; m.M[0][2] = Tool.S(angleRadians); m.M[0][3] = 0;
        m.M[1][0] = 0; m.M[1][1] = 1; m.M[1][2] = 0; m.M[1][3] = 0;
        m.M[2][0] = -Tool.S(angleRadians); m.M[2][1] = 0; m.M[2][2] = Tool.C(angleRadians); m.M[2][3] = 0;
        m.M[3][0] = 0; m.M[3][1] = 0; m.M[3][2] = 0; m.M[3][3] = 1;
        return m;

    @staticmethod
    def RotateAboutZAxis(angle_degrees):
          angleRadians = Tool.ToRadians(angle_degrees)
          m = TransformationMatrix()

          m.M[0][0] = Tool.C(angleRadians); m.M[0][1] = -Tool.S(angleRadians); m.M[0][2] = 0; m.M[0][3] = 0;
          m.M[1][0] = Tool.S(angleRadians); m.M[1][1] = Tool.C(angleRadians); m.M[1][2] = 0; m.M[1][3] = 0;
          m.M[2][0] = 0; m.M[2][1] = 0; m.M[2][2] = 1; m.M[2][3] = 0;
          m.M[3][0] = 0; m.M[3][1] = 0; m.M[3][2] = 0; m.M[3][3] = 1;

          return m;
    @staticmethod
    def EulerAngles( roll_alpha_ZDegrees, pitch_beta_Ydegrees, yaw_gamma_Xdegrees):
             return TransformationMatrix.RotateAboutZAxis(roll_alpha_ZDegrees).Multiply(TransformationMatrix.RotateAboutYAxis(pitch_beta_Ydegrees)).Multiply(TransformationMatrix.RotateAboutXAxis(yaw_gamma_Xdegrees))

    @staticmethod
    def rotateAboutArbitaryAxis(axis, angle_Degrees, position):
                axis.Normalize()
                kx = axis.X, ky = axis.Y, kz = axis.Z
                angle_radianhs = angle_Degrees * Math.PI / 180.0
                cT = Math.Cos(angle_radianhs)
                sT = Math.Sin(angle_radianhs)
                vT = 1 - cT

                r11 = kx * kx * vT + cT; r12 = kx * ky * vT - kz * sT; r13 = kx * kz * vT + ky * sT
                r21 = kx * ky * vT + kz * sT; r22 = ky * ky * vT + cT; r23 = ky * kz * vT - kx * sT
                r31 = kx * kz * vT - ky * sT; r32 = ky * kz * vT + kx * sT; r33 = kz * kz * vT + cT

                m = TransformationMatrix();

                m.M[0][0] = r11; m.M[0][1] = r12; m.M[0][2] = r13;
                m.M[1][0] = r21; m.M[1][1] = r22; m.M[1][2] = r23;
                m.M[2][0] = r31; m.M[2][1] = r32; m.M[2][2] = r33;

                m.M[0][3] = position.X
                m.M[1][3] = position.Y
                m.M[2][3] = position.Z

                m.M[3][0] = 0; m.M[3][1] = 0; m.M[3][2] = 0; m.M[3][3] = 1;

                return m
    
    SimplifyEulerAngles = False;
    @staticmethod 
    def ToEulerAngles( m):
                r11 = m.M[0][0]; r12 = m.M[0][1]; r13 = m.M[0][2];
                r21 = m.M[1][0]; r22 = m.M[1][1]; r23 = m.M[1][2];
                r31 = m.M[2][0]; r32 = m.M[2][1]; r33 = m.M[2][2];

                if (TransformationMatrix.SimplifyEulerAngles):
                
                    alpha = 0;
                    beta = 90;
                    gamma = atan2(r12, r22) * 180.0 / pi
                else:
                    b = atan2(-r31, sqrt(r11 * r11 + r21 * r21));
                    beta = b * 180.0 / pi
                    cB = cos(b)
                    alpha = atan2(r21 / cB, r11 / cB) * 180.0 / pi;
                    gamma = atan2(r32 / cB, r33 / cB) * 180.0 / pi;
                
                return alpha, beta, gamma

    @staticmethod
    def generateTransformationMatrix(DirectionVectorToAlignTo, StartingVector, positionXYZ):
                angle = Vector.AngleBetween(StartingVector, DirectionVectorToAlignTo)
                rotationAbout = Vector.CrossProduct(StartingVector, DirectionVectorToAlignTo)

                return TransformationMatrix.rotateAboutArbitaryAxis(rotationAbout, angle, positionXYZ)
