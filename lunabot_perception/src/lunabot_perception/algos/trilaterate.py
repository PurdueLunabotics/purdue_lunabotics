# Trilateration nonlinear diff. eq. test
# Based on Reliable computation of the points of intersection of n spheres in IR n by I.D. Coope
import numpy as np
import scipy.optimize as sci


# Function to minimize (from cited paper (Eqn 18)
def S(v, d1, d2, d3, a1, a2, a3):
    s = (
        (np.linalg.norm(v - a1) - d1) ** 2
        + (np.linalg.norm(v - a2) - d2) ** 2
        + (np.linalg.norm(v - a3) - d3) ** 2
    )
    return s


def is_full_rank(A):
    return np.linalg.matrix_rank(A) == A.shape[0]


class Trilaterate:
    n: int = 3

    def __init__(self, pts_cfg):
        assert pts_cfg.shape[0] == self.n
        assert pts_cfg.shape[1] == self.n
        self.pts_cfg = pts_cfg

    def trilaterate(self, D):
        """Solution by Orthogonal decomp
        #from scipy import linalg
        #from scipy.linalg import solve
        a_n = pts_cfg[-1]
        A = pts_cfg[:-1]
        A -= a_n
        A = A.T
        Q,R = linalg.qr(A)
        R = R[:-1]

        c = np.zeros(self.n-1)
        for i in range(self.n-1):
            c[i] = 0.5 * (D[-1]**2 - D[i]**2 + np.linalg.norm(R[i])**2)
        y = solve(R.T, c)
        z = np.sqrt(D[-1]**2 - np.linalg.norm(y)**2)
        v = np.zeros(self.n)
        v[:self.n-1] = y
        v[-1] = z

        x = Q @ v + a_n
        return x
        """

        pos = sci.minimize(
            S,
            [-5, -5, 0.5],
            args=(D[0], D[1], D[2], self.pts_cfg[0], self.pts_cfg[1], self.pts_cfg[2]),
            method="Powell",
        )
        return pos.x


# Find distances
def dist_from_pos(x, a1, a2, a3):
    d1 = np.linalg.norm(a1 - x)
    d2 = np.linalg.norm(a2 - x)
    d3 = np.linalg.norm(a3 - x)
    return (d1, d2, d3)


if __name__ == "__main__":

    error_accumulator = 0

    # Beacon positions
    a1 = np.array([0, 0.9, 0])  # x, y, z
    a2 = np.array([0.8, 0.0, 0])
    a3 = np.array([0, 0, 0])

    a1 = np.array([0, -0.876, 0])  # TX1
    a2 = np.array([0, 0, 0])  # TX2
    a3 = np.array([1.105, 0, 0])  # TX3

    pts_cfg = np.array([a1, a2, a3])

    tril = Trilaterate(pts_cfg)

    trials = 100

    running_error = np.zeros((trials))

    for i in range(0, trials):
        x = np.array([np.random.uniform(-5, -3), np.random.uniform(-5, -3), 0.6])
        d1, d2, d3 = dist_from_pos(x, a1, a2, a3)

        # Add noise to signals
        d1 += np.random.uniform(-0.01, 0.01)
        d2 += np.random.uniform(-0.01, 0.01)
        d3 += np.random.uniform(-0.01, 0.01)

        D = np.array([d1, d2, d3])
        pos = tril.trilaterate(D)
        # Perform optimizations
        error = np.linalg.norm(pos[:2] - x[:2]) ** 2

        print("Iteration", i, "X:", x, "Result", pos)
        print("err", error)
        running_error[i] = error

    print("Average error:", np.mean(running_error))
    print("mean squared error", np.mean(running_error ** 2))
