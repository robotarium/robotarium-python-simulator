import numpy as np


def uni_to_int(dxu, x, lambda_val):
    """
    Translates from single integrator to unicycle dynamics.

    Parameters
    ----------
    dxu :
        Single integrator control input.
    x :
        Unicycle states (3 x N)
    lambda_val :


    Returns
    -------
    dx :


    """
    n = dxu.shape[1]
    dx = np.zeros((2, n))

    for i in range(0, n):
        temp = np.array([[np.cos(x[2, i]), -lambda_val * np.sin(x[2, i])],
                         [np.sin(x[2, i]), lambda_val * np.cos(x[2, i])]])
        dx[:, i] = np.dot(temp, dxu[:, i])

    return dx
