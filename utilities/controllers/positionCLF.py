import numpy as np


def positionCLF(states, poses):
    """
    SUMMARY OF THIS FUNCTION GOES HERE.

    Parameters
    ----------
    states : SOMETHING
        SOMETHING
    poses : SOMETHING
        SOMETHING

    """
    N = states.shape[1]
    dx = np.zeros((2, N))

    for i in range(0, N):
        dx_temp = poses[0, i] - states[0, i]
        dy = poses[1, i] - states[1, i]
        dt = np.arctan2(dy, dx_temp)

        dist = np.sqrt(np.power(dx_temp, 2) + np.power(dy, 2))

        dx[0, i] = np.cos(dt - states[2, i])
        dx[1, i] = np.sin(dt - states[2, i])

    return dx
