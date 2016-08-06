import numpy as np


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

    L = np.zeros((v, v))

    for i in range(1, v):
        edge = np.random.randint()

        # Update adjacency relations.
        L[i, edge] = -1
        L[edge, i] = -1

        # Update node degrees
        L[i, i] += 1
        L[edge, edge] += 1

    # This works because all nodes have at least 1 degree. Choose from only
    # upper diagonal portion.
    potEdges = np.where(np.triu() == 1)
    sz = np.shape(L)

    numEdges = np.min(e, len(potEdges))

    if numEdges <= 0:
        return

    # Indices of randomly chosen extra edges.
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
