import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
a = [0, 0.4318, 0.0203, 0, 0, 0]
d = [0, 0, 0.15005, 0.4318, 0, 0]
alpha = [pi/2, 0, -pi/2, pi/2, -pi/2, 0]
qlim = [[-2.7925, 2.7925],[-0.7854, 3.9270],[-3.9270, 0.7854],[-1.9199, 2.9671],[-1.7453, 1.7453],[-4.6426, 4.6426]]
links = []

for i in range(6):
    link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
    links.append(link)
p560 = rtb.DHRobot(links, name ='Puma560')

qn = [0, 0.78539816, 3.14159265, 0, 0.78539816, 0]

target = SE3(0.6,-0.5,0.1)

mask = [1, 1, 1, 0, 0, 0]

solution = p560.ikine_LM(target, q0=qn, mask=mask)

print(f"IK Solution: {solution.q}")
