import numpy as np


def int_to_uni(dxi, x, lambda_val):
    """
    Translates from single integrator to unicycle dynamics.

    Parameters
    ----------
    dxi : something
        SOMETHING
    x : something
        Unicycle states 3 x N
    lambda_val :


    Returns
    -------
    dx : SOMETHING
        SOMETHING

    """
    n = dxi.shape[1]
    dx = np.zeros((2, n))
    t = np.array([[1, 0], [0, 1/lambda_val]])

    for i in range(0, n):
        temp = np.array([[np.cos(x[2, i]), np.sin(x[2, i])],
                         [-1*np.sin(x[2, i]), np.cos(x[2, i])]])
        dx[:, i] = t * np.dot(temp, dxi[:, i])

    return dx
