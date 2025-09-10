#Q9
from roboticstoolbox import trapezoidal
import numpy as np
from math import pi

q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6]
q2 = [-pi/10, -pi/7, -pi/5, -pi/3, -pi/4, -pi/6]
steps = 150

traj = trapezoidal(0.0, 1.0, steps)
interp_factor = traj.q[:, None]

joint_traj = (1 - interp_factor) * q1 + interp_factor * q2

joint_vel = np.diff(joint_traj, axis=0, prepend=joint_traj[[0]])

rounded_vel = np.round(joint_vel, 4)

peak_vel = np.abs(rounded_vel).max()

print(f"max vel: {peak_vel}")
