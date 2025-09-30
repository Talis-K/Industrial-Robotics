import numpy as np

q = np.deg2rad([30,-30,35,-30,0])

a = 1.0

cumulative_angle = 0
x = 0
y = 0

for theta in q:
    cumulative_angle += theta
    x += a * np.cos(cumulative_angle)
    y += a * np.sin(cumulative_angle)

print(f"End effector x position:{x:.3f}")
print("End effector y position:", y)
