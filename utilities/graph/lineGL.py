import numpy as np


def lineGL(n):
    """
    SUMMARY OF THE FUNCTION GOES HERE.

    Parameters
    ----------
    n : int
        SOMETHING

    """
    laplacian = 2 * np.eye(n) - np.diag(np.ones(1, n-1), 1) - \
        np.diag(np.ones(1, (n-1)), -1)
    laplacian[0, 0] = 1
    laplacian[n-1, n-1] = 1

    return laplacian
