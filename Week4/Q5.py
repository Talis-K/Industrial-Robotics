import numpy as np

q = np.array([-0.7506, 0.5195, -1.8286, 2.5971])
jointlim = np.pi

joint2q = q[1]

dist2upper = abs(jointlim - joint2q)
dist2lower = abs(-jointlim - joint2q)

closestdist = min(dist2upper, dist2lower)

print(closestdist)
