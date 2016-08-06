import numpy as np


def park(states, poses, gamma, k, h):
    """
    SUMMARY OF THIS FUNCTION GOES HERE.

    Parameters
    ----------
    states : SOMETHING
        SOMETHING
    poses : SOMETHING
        SOMETHING
    gamma : SOMETHING
        SOMETHING
    k : SOMETHING
        SOMETHING
    h : SOMETHING
        SOMETHING

    """
    N = states.shape[1]
    dxu = np.zeros((2, N))

    for i in range(0, N):
        translate = R(-1*poses[2, i]) * (poses[0:2, i] - states[0:2, i])
        e = np.linalg.norm(translate)
        theta = np.arctan2(translate[1], translate[0])
        alpha = theta - (states[2, i] - poses[2, i])
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        ca = np.cos(alpha)
        sa = np.sin(alpha)

        dxu[0, i] = gamma * e * ca
        dxu[1, i] = k * alpha + gamma * ((ca*sa)/alpha) * (alpha + h * theta)

    return dxu


def R(x):
    """ Create Rotation Matrix. """
    return np.array([[np.cos(x), -1*np.sin(x)], [np.sin(x), np.cos(x)]])


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


def positionInt(states, poses, lambda):
    """
    Position controller via single integrator dynamics.

    Parameters
    ----------
    states : SOMETHING
        SOMETHING
    poses : SOMETHING
        SOMETHING
    lambda : SOMETHING
        SOMETHING

    """
    N = poses.shape[1]
    dx = np.zeros((2, N))

    for i in range(0, N):
        dx[:, i] = poses[0:2, i] - states[0:2, i]

    return dx
