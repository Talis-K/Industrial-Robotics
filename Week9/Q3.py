import numpy as np

# Define grid
X, Y = np.meshgrid(np.arange(-5, 5.1, 0.1), np.arange(-5, 5.1, 0.1))
Z = X  # Z = X

# Flatten arrays for easier calculation
x = X.flatten()
y = Y.flatten()
z = Z.flatten()

# Ellipsoid parameters
center = np.array([0, 0, 10])
radii = np.array([11, 11, 11])

# Compute ellipsoid equation
inside = (((x - center[0]) / radii[0]) ** 2 +
          ((y - center[1]) / radii[1]) ** 2 +
          ((z - center[2]) / radii[2]) ** 2) <= 1

# Count points inside
count_inside = np.sum(inside)

print("Number of points inside ellipsoid:", count_inside)
