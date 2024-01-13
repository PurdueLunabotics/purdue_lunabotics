# Trilateration nonlinear diff. eq. test
# Based on Reliable computation of the points of intersection of n spheres in IR n by I.D. Coope
import numpy as np
import scipy.optimize as sci

# Beacon positions
a1 = np.array([0, 1, 0])  # x, y, z
a2 = np.array([-1, 0, 0])
a3 = np.array([1, 0, 0])


# Find distances
def dist_from_pos(x, a1, a2, a3):
    d1 = np.linalg.norm(a1 - x)
    d2 = np.linalg.norm(a2 - x)
    d3 = np.linalg.norm(a3 - x)
    return (d1, d2, d3)


# Function to minimize (from cited paper)
def S(v, d1, d2, d3):
    s = (
        (np.linalg.norm(v - a1) - d1) ** 2
        + (np.linalg.norm(v - a2) - d2) ** 2
        + (np.linalg.norm(v - a3) - d3) ** 2
    )
    return s


error_accumulator = 0

for i in range(0, 100):
    x = np.array(
        [
            np.random.uniform(0, 10),
            np.random.uniform(0, 10),
            np.random.uniform(-0.1, 0.1),
        ]
    )
    d1, d2, d3 = dist_from_pos(x, a1, a2, a3)
    # Add noise to signals
    d1 += np.random.uniform(-0.1, 0.1)
    d2 += np.random.uniform(-0.1, 0.1)
    d3 += np.random.uniform(-0.1, 0.1)
    # Perform optimizations
    pos = sci.minimize(S, [0, 10, 0], args=(d1, d2, d3), method="Powell")
    error = np.linalg.norm(pos.x - x) ** 2
    if error > 1:
        print("Iteration", i, "X:", x, "Result", pos.x)
        print(error)
    error_accumulator += error

print("Mean Squared Error", np.sqrt(error_accumulator) / 1000)
