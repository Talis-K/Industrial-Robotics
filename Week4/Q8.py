import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
import numpy as np

left = rtb.models.DH.Baxter('left')
right = rtb.models.DH.Baxter('right')   

left.base = SE3(0.064614, 0.25858, 0.119) * SE3.Rx(pi/4)
right.base = SE3(0.063534, -0.25966, 0.119)* SE3.Rx(-pi/4)

qleft = np.zeros(7)
qright = np.zeros(7)

left_teach = left.fkine(qleft)
right_teach = right.fkine(qright)

left_pos = left_teach.t 
right_pos = right_teach.t  

distance = 1000* ((left_pos[0] - right_pos[0])**2 + 
            (left_pos[1] - right_pos[1])**2 + 
            (left_pos[2] - right_pos[2])**2)**0.5

print(distance)