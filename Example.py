import RobotArmFKIK as rk
import numpy as np
from time import sleep
from TransformationMatrix import TransformationMatrix

L1=18
L2=26
L3=6 + 10
r = rk.Kinematics.RobotArmMechanism()
r.addSegment("None",0, 0.0, 0.0, 0)
r.addSegment("RotZ", 0, -90, 0, 0)
r.addSegment("RotZ", L1, 0.0, 0, 0)
r.addSegment("RotZ", L2, 0.0, 0, 0)
r.addSegment("RotZ", L3, 90.0, 0, 0)
r.addSegment("RotZ")

r.finaliseChain()
#x = [0.0,-90.0 +(0.0),90.0 + (0.0),0.0,0.0]
x=[ -30.0, -30.0, 50.0, 0.0,0.0]
x = [0.0,0.0,0.0,90.0, 0.0]
print(r.computeForwardKinematics(x))
y = [[ 0, 0, 1.0, 37], [0,1,0,8],[-1.0,0,0,-33],[0,0,0,1]]
guess = [30.0, -90.0, 90.0, 0.0, 0.0]
angles = np.array([0.0,0.0,0.0,0.0,0.0])
while(True):
   print(r.computeInverseKinematics(y, x, angles))
   x = list(angles)
   angles[1] = angles[1]+ 90.0
   angles[2] = angles[2] - 90.0
   print(angles)
   changed = False
   for i in range(0, 5):
       if(angles[i] < -90.0):
            x[i] = 45.0
            changed = True
       if(angles[i] > 90.0):
           x[i] = -45.0
           changed = True          
   if(changed == False):
        break
print(r)
alpha = -0.0
beta = 90.0
gamma = 0.0
T = TransformationMatrix.EulerAngles(alpha, beta, gamma)
print(T.M)
T1 = TransformationMatrix()
T1.M[0][0] = 0.0; T1.M[0][1] = 0.0; T1.M[0][2] = 1.0;
T1.M[1][0] = 0.0; T1.M[1][1] = 1.0; T1.M[1][2] = 0.0;
T1.M[2][0] = -1.0; T1.M[2][1] = 0.0; T1.M[2][2] = 0.0;
print(TransformationMatrix.ToEulerAngles(T1))

