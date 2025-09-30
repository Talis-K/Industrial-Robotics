from roboticstoolbox import DHLink, DHRobot
from math import pi

l1 = DHLink(d=1, a=0, alpha =pi/2, offset=0)
l2 = DHLink(d=0, a=2, alpha =0, offset=0)
l3 = DHLink(d=0, a=3, alpha =0, offset=0)
R3 = DHRobot([l1,l2,l3])

q = [pi/12, 0, 0]

print([R3.fkine(q)])