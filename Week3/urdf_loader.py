from roboticstoolbox import Robot
from pathlib import Path
import numpy as np
import swift
import time

urdf_path = Path(__file__).parent / "myrobotExample.urdf"
robot = Robot.URDF(str(urdf_path))

env = swift.Swift()
env.launch()
env.add(robot)

robot.q = np.zeros(robot.n)

t0 = time.time()
while True:
    t = time.time() - t0
    robot.q = 0.5 * np.sin(t) * np.ones(robot.n)
    env.step(0.02)