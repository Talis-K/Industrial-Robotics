from roboticstoolbox import DHLink, DHRobot
from math import pi
import numpy as np

l1 = DHLink(d=0, a=1, alpha =0, offset=0)
l2 = DHLink(d=0, a=1, alpha =0, offset=0)
l3 = DHLink(d=0, a=1, alpha =0, offset=0)
l4 = DHLink(d=0, a=1, alpha =0, offset=0)
l5 = DHLink(d=0, a=1, alpha =0, offset=0)
R3 = DHRobot([l1,l2,l3,l4,l5])

q = np.deg2rad([60,-45,35,-60,0])

print([R3.fkine(q)])