import numpy as np


def cycleGL(n):
    """
    Generates a graph Laplacian for a cycle graph.
    The order is assumed to be 1->2->3->...->n

    Parameters
    ----------
    n : int
        SOMETHING

    """
    L = 2 * np.eye(n) - np.diag(np.ones(1, (n-1)), 1) - \
        np.diag(np.ones(1, (n-1)), -1)
    L[n-1, 0] = -1
    L[0, n-1] = -1

    return L
