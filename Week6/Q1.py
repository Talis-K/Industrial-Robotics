import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi
left = rtb.models.DH.Baxter('left')
right = rtb.models.DH.Baxter('right')
left.base = SE3(0.064614, 0.25858, 0.119) * SE3.Rx(pi/4)
right.base = SE3(0.063534, -0.25966, 0.119) * SE3.Rx(-pi/4)

qLeft = [pi/6, 0, 0, 0, 0, 0, -3*pi/2]
qRight = [-9*pi/10, 0, 0, 4*pi/9, 0, 0, 0]

leftee = left.fkine(qLeft)
rightee = right.fkine(qRight)

left_pos = leftee.t
right_pos = rightee.t

distance = np.linalg.norm(left_pos - right_pos)

print("Distance between the end-effectors:", distance)

