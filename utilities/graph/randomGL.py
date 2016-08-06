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
    L = np.tril(np.ones((v, v)))

    # This works because I can't select diagonals
    potEdges = np.where(np.triu(L) == 0)
    sz = L.shape

    # Rest to zeros
    L = np.zeros((v, v))

    numEdges = np.min(e, len(potEdges))
    edgeIndices = np.random.randperm(len(potEdges), numEdges)

    for index in edgeIndices:
        # FIX THIS
        # [i, j] = ind2sub(sz, potEdges[index])

        # Update adjacency relation
        L[i, j] = -1
        L[j, i] = -1

        # Update degree relation
        L[i, i] += 1
        L[j, j] += 1
