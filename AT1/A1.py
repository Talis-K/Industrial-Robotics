from ir_support import LinearUR3
import swift
import spatialgeometry
from spacialmathgeometry import Cuboid

# create and launch a swift environment
env = swift.Swift()
env.launch()

# create an UR5 on linear rail and add it into environment
r = LinearUR3()
r.add_to_env(env)

# get the end-effector transform and print it out
tr = r.fkine(r.q).A
print(tr)

# disired variables
desired_x = 0.5
desired_y = 0.0
desired_z = 0.5

# Compute desired pose (same orientation as current, new position)
desired_T = r.fkine(r.q)
desired_T.t = [desired_x, desired_y, desired_z]

# Compute inverse kinematics
sol = r.ikine_LM(desired_T)

x = cuboid

if sol.success:
    print("IK solution joint angles:", sol.q)
    r.q = sol.q  # Update the robot's configuration to the new joint angles
    env.step(0)  # Refresh the visualization (0 timestep for instant update)
else:
    print("No IK solution found (may be unreachable or require mask/constraints)")

# hold the environment to see the result
env.hold()

# get brick to follow a transfor that is teh same as the end effector