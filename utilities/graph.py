import numpy as np


def completeGL(n):
    """
    SUMMARY OF THIS FUNCTION GOES HERE.

    Parameters
    ----------
    n : int
        SOMETHING

    """
    return n * np.eye(n) - np.ones((n, n))


def cycleGL(n):
    """
    Generates a graph Laplacian for a cycle graph.
    The order is assumed to be 1->2->3->...->n

    Parameters
    ----------
    n : int
        SOMETHING

    """
    laplacian = 2 * np.eye(n) - np.diag(np.ones(1, (n-1)), 1) - \
        np.diag(np.ones(1, (n-1)), -1)
    laplacian[n-1, 0] = -1
    laplacian[0, n-1] = -1

    return laplacian


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


def randomConnectedGL(v, e):
    """
    Outputs a randomly generated, undirected, connected graph.
    Laplacian with v - 1 + e edges

    Parameters
    ----------
    v : SOMETHING
        SOMETHING
    e : SOMETHING
        SOMETHING

    """

    laplacian = np.zeros((v, v))

    for i in range(1, v):
        edge = np.random.randint(i)

        # Update adjacency relations.
        laplacian[i, edge] = -1
        laplacian[edge, i] = -1

        # Update node degrees
        laplacian[i, i] += 1
        laplacian[edge, edge] += 1

    # This works because all nodes have at least 1 degree. Choose from only
    # upper diagonal portion.
    pot_edges = np.where(np.triu() == 1)
    sz = np.shape(laplacian)

    num_edges = np.min(e, len(pot_edges))

    if num_edges <= 0:
        return

    # Indices of randomly chosen extra edges.
    edge_indices = np.random.permutation(len(pot_edges), num_edges)

    for index in edge_indices:
        # FIX THIS
        i, j = ind_to_sub(sz, pot_edges[index])

        # Update adjacency relation
        laplacian[i, j] = -1
        laplacian[j, i] = -1

        # Update degree relation
        laplacian[i, i] += 1
        laplacian[j, j] += 1

    return laplacian


def randomGL(v, e):
    """
    Outputs a randomly generated, undirected, connected graph Laplacian with
    'n' nodes.

    Parameters
    ----------
    v : SOMETHING
        SOMETHING
    e : SOMETHING
        SOMETHING

    """
    laplacian = np.tril(np.ones((v, v)))

    # This works because I can't select diagonals
    pot_edges = np.where(np.triu(laplacian) == 0)
    sz = laplacian.shape

    # Rest to zeros
    laplacian = np.zeros((v, v))

    num_edges = np.min(e, len(pot_edges))
    edge_indices = np.random.permutation(len(pot_edges), num_edges)

    for index in edge_indices:
        # FIX THIS
        i, j = ind_to_sub(sz, pot_edges[index])

        # Update adjacency relation
        laplacian[i, j] = -1
        laplacian[j, i] = -1

        # Update degree relation
        laplacian[i, i] += 1
        laplacian[j, j] += 1

    return laplacian
