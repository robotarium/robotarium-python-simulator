import numpy as np


def completeGL(n):
    """
    SUMMARY OF THIS FUNCTION GOES HERE.

    Parameters
    ----------
    n : int
        SOMETHING

    """
    return (n * np.eye(n) - np.ones((n, n)))
