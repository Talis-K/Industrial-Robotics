from roboticstoolbox import jtraj, trapezoidal, mtraj, mstraj, ctraj
from math import pi

q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6]
q2 = [-pi/10, pi/7, -pi/5, -pi/3, -pi/4, -pi/6]

traj = jtraj(q1, q2, 50)

print(traj.q) 