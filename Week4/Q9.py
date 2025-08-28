import numpy as np
from roboticstoolbox import jtraj, trapezoidal
from math import pi

steps = 100 

q1 = np.array([-0.4382, 0.2842, -0.7392, -3.142, -0.455, -2.703])
q2 = np.array([0.9113, -0.5942, -0.01781, 0, 0.612, -0.9113])
quin = jtraj(q1, q2, steps)
quinrel = np.diff(quin.q[:, 0])
trapezoid = trapezoidal(0, 1, steps)
mattrip = np.empty((steps, 6))



for i in range(steps):
    mattrip[i, :] = (1 - trapezoid.q[i]) * q1 + trapezoid.q[i] * q2
traprel = np.diff(mattrip[:, 0])
deltav = np.abs(np.abs(quinrel) - np.abs(traprel))
maxdifference = np.max(deltav)

print(maxdifference)