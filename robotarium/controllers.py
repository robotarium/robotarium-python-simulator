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
    n = states.shape[1]
    dxu = np.zeros((2, n))

    for i in range(0, n):
        translate = rot_mat(-1*poses[2, i]) * (poses[0:2, i] - states[0:2, i])
        e = np.linalg.norm(translate)
        theta = np.arctan2(translate[1], translate[0])
        alpha = theta - (states[2, i] - poses[2, i])
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        ca = np.cos(alpha)
        sa = np.sin(alpha)

        dxu[0, i] = gamma * e * ca
        dxu[1, i] = k * alpha + gamma * ((ca*sa)/alpha) * (alpha + h * theta)

    return dxu


def rot_mat(x):
    """ Create Rotation Matrix to rotate a 2d numpy array by the angle x
    
    Parameters
    ----------
    x : float
        angle defining the rotation (rotation is counter clockwise for positive
        values)

    Returns 
    -------
    mat : np.ndarray
        a 2 x 2 matrix that can be used to rotate a 2d array
 
    Examples 
    --------
    >>> import numpy as np
    >>> from robotarium.controllers import rot_mat
    >>> rot = rot_mat(np.pi / 2)
    >>> rot.dot(np.array([1, 0]))
    array([  6.12323400e-17,   1.00000000e+00])
    """
    return np.array([[np.cos(x), -1*np.sin(x)], [np.sin(x), np.cos(x)]])


def position_clf(states, poses):
    """
    SUMMARY OF THIS FUNCTION GOES HERE.

    Parameters
    ----------
    states : SOMETHING
        SOMETHING
    poses : SOMETHING
        SOMETHING

    """
    n = states.shape[1]
    dx = np.zeros((2, n))

    for i in range(0, n):
        dx_temp = poses[0, i] - states[0, i]
        dy = poses[1, i] - states[1, i]
        dt = np.arctan2(dy, dx_temp)

        dist = np.sqrt(np.power(dx_temp, 2) + np.power(dy, 2))

        dx[0, i] = np.cos(dt - states[2, i])
        dx[1, i] = np.sin(dt - states[2, i])

    return dx


def position_int(states, poses, lambda_val):
    """
    Position controller via single integrator dynamics.

    Parameters
    ----------
    states : np.ndarray
        a 5 x n numpy array with rows representing x position, y position,
        angular position, forward velocity, and angular velocity. The kth
        column represents the state for the kth vehicle.

    poses : np.ndarray
        a 3 x n numpy array with rows representing desired x position, y
        position, and angular position.

    lambda_val : float
        gain

    """
    n = poses.shape[1]
    dx = np.zeros((2, n))

    for i in range(0, n):
        dx[:, [i]] = poses[0:2, [i]] - states[0:2, [i]]

    return dx
