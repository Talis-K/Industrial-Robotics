import numpy as np
from roboticstoolbox import DHLink, DHRobot
from math import pi

link1 = DHLink(d= 0.5, a= 0.3, alpha= np.deg2rad(90), qlim= [-pi, pi]) 
link2 = DHLink(d= 0, a= 1, alpha= 0, qlim= [-pi, pi]) 
link3 = DHLink(d= 0, a= 0.2, alpha= -np.deg2rad(90), qlim= [-pi, pi]) 
link4 = DHLink(d= 0.7, a= 0, alpha= np.deg2rad(90), qlim= [-pi, pi])
robot = DHRobot([link1, link2, link3, link4])

q = [-0.7506, 0.5895, -1.8286,  2.5971] 
robot.q = q

T = robot.fkine(q)
print(T)