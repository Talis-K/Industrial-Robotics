import numpy as np
import roboticstoolbox as rtb

# Create UR5 robot
r = rtb.models.DH.UR5()

# Define joint configuration in degrees
q_deg = [0,55,-90,-45,90,0]

# Convert to radians
q_rad = np.radians(q_deg)

# Compute manipulability for translational motion
manip_trans = r.manipulability(q_rad, axes='trans')

# Print result to 4 decimal places
print(f"Translational manipulability: {manip_trans:.4f}")
