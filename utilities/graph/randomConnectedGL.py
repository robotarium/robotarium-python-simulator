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
        i, j = ind2sub(sz, pot_edges[index])

        # Update adjacency relation
        laplacian[i, j] = -1
        laplacian[j, i] = -1

        # Update degree relation
        laplacian[i, i] += 1
        laplacian[j, j] += 1

    return laplacian
