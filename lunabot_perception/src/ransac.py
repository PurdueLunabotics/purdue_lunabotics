from copy import copy
import numpy as np
from numpy.random import default_rng

rng = default_rng()

# thank you wikipedia

class RANSAC:
    def __init__(self, n=10, k=1, t=0.05, d=10, model=None, loss=None, metric=None):
        self.n = n              # `n`: Minimum number of data points to estimate parameters
        self.k = k              # `k`: Maximum iterations allowed
        self.t = t              # `t`: Threshold value to determine if points are fit well
        self.d = d              # `d`: Number of close data points required to assert model fits well
        self.model = model      # `model`: class implementing `fit` and `predict`
        self.loss = loss        # `loss`: function of `y_true` and `y_pred` that returns a vector
        self.metric = metric    # `metric`: function of `y_true` and `y_pred` and returns a float
        self.best_fit = None
        self.best_error = np.inf

    def fit(self, X, y):
        # try:
        X = np.array(X)
        y = np.array(y)
        print(y)
        print(f"x dimensions: {X.shape}")
        print(f"y dimensions: {y.shape}")
        print(f"fit iterations: {self.k}")
        for _ in range(self.k):
            
            ids = rng.permutation(X.shape[0])

            maybe_inliers = ids[: self.n]
            # print(f"x: {X.head()}")
            print(f"y: {y[maybe_inliers]}")
            print(f"mi: {maybe_inliers}")
            print(f"x: {X[maybe_inliers]}")
            print(f"weeeeeeeeeee")
            
            maybe_model = copy(self.model).fit(X[maybe_inliers], y[maybe_inliers])

            thresholded = (
                self.loss(y[ids][self.n :], maybe_model.predict(X[ids][self.n :]))
                < self.t
            )

            inlier_ids = ids[self.n :][np.flatnonzero(thresholded).flatten()]

            if inlier_ids.size > self.d:
                inlier_points = np.hstack([maybe_inliers, inlier_ids])
                better_model = copy(self.model).fit(X[inlier_points], y[inlier_points])

                this_error = self.metric(
                    y[inlier_points], better_model.predict(X[inlier_points])
                )

                if this_error < self.best_error:
                    self.best_error = this_error
                    self.best_fit = better_model
                print("oops")

        return self
        # except Exception as e:
        #     print(f"Error: {e}")

    def predict(self, X: np.ndarray):
        if (type(X) != np.ndarray):
            X = np.array(X)
        return self.best_fit.predict(X)

def square_error_loss(y_true: np.ndarray, y_pred: np.ndarray):
    return (y_true - y_pred) ** 2


def mean_square_error(y_true: np.ndarray, y_pred: np.ndarray):
    return np.sum(square_error_loss(y_true, y_pred)) / len(y_true)

class LinearRegressor:
    def __init__(self):
        self.params = None

    def fit(self, X: np.ndarray, y: np.ndarray):
        r = X.shape[0]
        X = np.hstack([np.ones((r, 1)), X])
        self.params = np.linalg.inv(X.T @ X) @ X.T @ y
        return self

    def predict(self, X: np.ndarray):
        r = X.shape[0]
        X = np.hstack([np.ones((r, 1)), X])
        return X @ self.params