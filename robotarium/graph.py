''' a set of utilities to create matrix representations (Laplacians) of graphs
'''
import numpy as np


def complete_gl(n):
    """
    return the Laplacian of a complete graph (all nodes are connected to all
    edges)

    Parameters
    ----------
    n : int
        number of nodes in the graph

    Examples
    --------
    >>> from robotarium.graph import complete_gl
    >>> complete_gl(4)
    array([[ 3., -1., -1., -1.],
           [-1.,  3., -1., -1.],
           [-1., -1.,  3., -1.],
           [-1., -1., -1.,  3.]])

    """
    return n * np.eye(n) - np.ones((n, n))


def cycle_gl(n):
    """
    return the Laplacian of a cycle graph (The order is assumed to be
    1->2->3->...->n)

    Parameters
    ----------
    n : int
        number of nodes in the graph

    Examples 
    -------- 
    >>> from robotarium.graph import cycle_gl
    >>> cycle_gl(4)
    array([[ 2., -1.,  0., -1.],
           [-1.,  2., -1.,  0.],
           [ 0., -1.,  2., -1.],
           [-1.,  0., -1.,  2.]])
    """
    laplacian = 2 * np.eye(n) - np.diag([1] * (n-1), 1) - \
        np.diag([1] * (n-1), -1)
    laplacian[n-1, 0] = -1
    laplacian[0, n-1] = -1

    return laplacian


def line_gl(n):
    """
    return the Laplacian of a line graph

    Parameters
    ----------
    n : int
        number of nodes in the graph

    Examples
    --------
    >>> from robotarium.graph import line_gl
    >>> line_gl(4)
    array([[ 1., -1.,  0.,  0.],
           [-1.,  2., -1.,  0.],
           [ 0., -1.,  2., -1.],
           [ 0.,  0., -1.,  1.]])
    """
    laplacian = 2 * np.eye(n) - np.diag(np.ones(n-1), 1) - \
        np.diag(np.ones(n-1), -1)
    laplacian[0, 0] = 1
    laplacian[n-1, n-1] = 1

    return laplacian


def random_connected_gl(v, e):
    """
    Outputs a randomly generated, undirected, connected graph.
    Laplacian with v - 1 + e edges

    Parameters
    ----------
    v : int
        number of nodes
    e : int
        number of edges

    Examples 
    --------

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
    temp = np.where(np.triu(laplacian).reshape(v*v) == 1)
    pot_edges = temp[0]
    sz = laplacian.shape

    # num_edges = min(e, len(pot_edges))
    num_edges = np.where(e <= len(pot_edges), e, len(pot_edges))

    if num_edges <= 0:
        return

    # Indices of randomly chosen extra edges.
    temp = np.random.permutation(len(pot_edges))
    edge_indices = temp[0:num_edges]

    i, j = ind_to_sub(sz, pot_edges[edge_indices])

    # Update adjacency relation
    laplacian[i, j] = -1
    laplacian[j, i] = -1

    # Update degree relation
    laplacian[i, i] += 1
    laplacian[j, j] += 1

    return laplacian


def random_gl(v, e):
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
    temp = np.where(np.triu(laplacian).reshape(v*v) == 0)
    pot_edges = temp[0]
    sz = laplacian.shape

    # Rest to zeros
    laplacian = np.zeros((v, v))

    num_edges = np.where(e <= len(pot_edges), e, len(pot_edges))

    # Indices of randomly chosen extra edges.
    temp = np.random.permutation(len(pot_edges))
    edge_indices = temp[0:num_edges]

    i, j = ind_to_sub(sz, pot_edges[edge_indices])

    # Update adjacency relation
    laplacian[i, j] = -1
    laplacian[j, i] = -1

    # Update degree relation
    laplacian[i, i] += 1
    laplacian[j, j] += 1

    return laplacian


def ind_to_sub(siz, ind):
    """
    Subscripts from linear index.

    This is a python formulation of the function ind2sub().
    The original function can be found here:
        https://www.mathworks.com/help/matlab/ref/ind2sub.html

    The function provided below is a modification of a function provided here:
    https://stackoverflow.com/questions/28995146/matlab-ind2sub-equivalent-in-python

    The subtraction by one in the 'rows' variable is to keep index changes
    consistent with a 0 index start compared to MATLAB's 1 start.

    Parameters
    ----------
    siz : int tuple
        contains the size of the matrix that is passed through.

    ind : np.ndarray
        the matrix that the linear index subscripts will be derived.

    Returns
    -------
    rows : np.ndarray
        vector containing the equivalent row subscripts corresponding to each
        linear index from the original matrix ind.

    columns : np.ndarray
        vector containing the equivalent column subscripts corresponding to each
        linear index from the original matrix ind.

    """
    ind[ind < 0] = -1
    ind[ind >= siz[0] * siz[1]] = -1
    rows = np.asarray((np.ceil(ind.astype('int') / siz[0]) - 1), dtype=int)
    columns = (ind % siz[1])
    return rows, columns
