from roboticstoolbox import DHLink, DHRobot
from math import pi

link1 = DHLink(d=0.1519, a=0 , alpha= pi/2, qlim=[-pi, pi])
link2 = DHLink(d=0, a= -0.24365, alpha= 0,  qlim=[-pi, pi])
link3 = DHLink(d=0, a= -0.21325, alpha= 0, qlim=[-pi, pi])
link4 = DHLink(d= 0.11235, a= 0, alpha= pi/2, qlim=[-pi, pi])
link5 = DHLink(d= 0.08535, a= 0, alpha= -pi/2, qlim=[-pi, pi])
link6 = DHLink(d= 0.0819, a= 0, alpha= 0, qlim=[-pi, pi])

robot = DHRobot([link1, link2, link3, link4, link5, link6])


q = [0, pi/10, 0, 0, 0, 0] 
robot.q = q

T = robot.fkine(q)
print(T)

