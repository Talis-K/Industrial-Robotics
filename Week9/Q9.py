import roboticstoolbox as rtb
import numpy as np
import sys
import os
from ir_support import RectangularPrism

# Add Week5 folder to Python path
week5_path = os.path.abspath("Week5")
sys.path.append(week5_path)

# Import is_collision from Week5
from lab5_solution_question2and3 import is_collision

p3 = rtb.models.DH.Planar3()
prism = RectangularPrism(1, 2.2, 2, center=[2.5, 0, 0])
vertex, faces, face_normals = prism.get_data()

# Start and end joint configs
q1 = np.array([np.pi/3, 0, 0])
q2 = np.array([np.pi/3, 0, 0])

# Generate 50-step trajectory
traj = rtb.jtraj(q1, q2, 50).q

# Run collision detection
result = is_collision(p3, traj, vertex, faces, face_normals, return_once_found=True)

# Find first collision
collision_index = np.where(result)[0][0] if np.any(result) else None

if collision_index is not None:
    print(f"First collision occurs at step {collision_index}")
    print(f"Joint configuration (q) = {traj[collision_index]}")
else:
    print("No collision detected along the trajectory.")
