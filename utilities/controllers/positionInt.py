import numpy as np


def positionInt(states, poses):
    """
    Position controller via single integrator dynamics.

    Parameters
    ----------
    states : SOMETHING
        SOMETHING
    poses : SOMETHING
        SOMETHING

    """
    N = poses.shape[1]
    dx = np.zeros((2, N))

    for i in range(0, N):
        dx[:, i] = poses[0:2, i] - states[0:2, i]

    return dx
