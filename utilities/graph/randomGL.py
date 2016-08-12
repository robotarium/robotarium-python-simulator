import numpy as np


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
        i, j = ind2sub(sz, pot_edges[index])

        # Update adjacency relation
        laplacian[i, j] = -1
        laplacian[j, i] = -1

        # Update degree relation
        laplacian[i, i] += 1
        laplacian[j, j] += 1

    return laplacian
