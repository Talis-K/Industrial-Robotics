import numpy as np
from roboticstoolbox import DHRobot, DHLink
from math import pi

l1 = DHLink(d=1, a=0, alpha=pi/2, offset=0)
l2 = DHLink(d=0, a=2, alpha=0,    offset=0)
l3 = DHLink(d=0, a=3, alpha=0,    offset=0)
R3 = DHRobot([l1, l2, l3])
R2 = DHRobot([l1, l2]) 

q = [pi/12, 0, 0]

p2 = R2.fkine(q[:2]).t      
pe = R3.fkine(q).t

p2 = np.array(p2).flatten()
pe = np.array(pe).flatten()

direction = pe - p2

x_plane = 3.2
t = (x_plane - p2[0]) / direction[0]   # parametric equation

p_intersect = p2 + t * direction
print("Intersection with wall:", p_intersect)