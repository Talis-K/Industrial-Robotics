import scipy.io as sp
import numpy as np
import roboticstoolbox as rtb

mat_contents = sp.loadmat("ur3_q.mat")
print(mat_contents.keys())
q_data = np.array(mat_contents['q'])
print(q_data)
print(q_data.shape)
print(q_data.dtype)
print("Min joint angles:", np.min(q_data, axis=0))
print("Max joint angles:", np.max(q_data, axis=0))