import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi

left = rtb.models.DH.Baxter('left')
right = rtb.models.DH.Baxter('right')   

left.base = SE3(0.064614, 0.25858, 0.119) * SE3.Rx(pi/4)
right.base = SE3(0.063534, -0.25966, 0.119)* SE3.Rx(-pi/4)

qleft = [0, pi/10, 0, 0, 0, -pi/10, 0]
qright = [0, -pi/10, 0, 0, 0, pi/10, 0]

# Compute forward kinematics for left and right arms
left_teach = left.fkine(qleft)
right_teach = right.fkine(qright)

# Extract end effector positions
left_pos = left_teach.t  # Position of left end effector
right_pos = right_teach.t  # Position of right end effector

for i in range(3):
    print(f"Left end effector position axis {i}: {left_pos[i]:.4f}")
    print(f"Right end effector position axis {i}: {right_pos[i]:.4f}")

# Calculate Euclidean distance between end effectors
distance = ((left_pos[0] - right_pos[0])**2 + 
            (left_pos[1] - right_pos[1])**2 + 
            (left_pos[2] - right_pos[2])**2)**0.5

# Convert to millimeters and round to the nearest whole number
distance_mm = round(distance * 1000)

print(f"Distance between end effectors: {distance_mm} mm")