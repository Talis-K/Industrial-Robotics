








# not working






import numpy as np
import roboticstoolbox as rtb

# Load Puma560
p560 = rtb.models.DH.Puma560()

# Explicitly set the joint limits (as you provided)
p560.qlim = np.row_stack((
    [-2.7925, -0.7854, -3.9270, -1.9199, -1.7453, -4.6426],
    [ 2.7925,  3.9270,  0.7854,  2.9671,  1.7453,  4.6426]
))

# Candidate configurations (q1..q4) from the question (radians)
q_list = [
    [0, 0.7, 3, 0, 0.7, 0], 
    [1.1170, 1.0996, -3.4872, 0.1466, 0.5585, 0.6500],
    [0, 2.3562, -3.0159, 0, -0.9076, 0],
    [0, 1.5708, -3.0159, 0.1466, 0.5585, 0]
]

best = None
best_manip = float('inf')

for i, q in enumerate(q_list, start=1):
    in_limits = np.all(q >= p560.qlim[0]) and np.all(q <= p560.qlim[1])
    manip_trans = p560.manipulability(q, axes='trans')
    print(f"q{i}: {q}")
    print(f"  within_limits: {in_limits}")
    print(f"  translational manipulability: {manip_trans:.6e}\n")

    if in_limits and manip_trans < best_manip:
        best_manip = manip_trans
        best = (i, q, manip_trans)

if best is None:
    print("No candidate is within the joint limits.")
else:
    idx, q_best, m_best = best
    print(f"Pose closest to singularity (lowest translational manipulability) is option q{idx}:")
    print(f"  q{idx} = {q_best}, manipulability = {m_best:.6e}")
