import numpy as np


def lineGL(n):
    """
    SUMMARY OF THE FUNCTION GOES HERE.

    Parameters
    ----------
    n : int
        SOMETHING

    """
    L = 2 * np.eye(n) - np.diag(np.ones(1, (n-1)), 1) - \
        np.diag(np.ones(1, (n-1)), -1)
    L[0, 0] = 1
    L[n-1, n-1] = 1

    return L
