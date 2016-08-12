import numpy as np


def barrierCertificate(dxi, x, safety_radius):
    """ Wraps single-integrator dynamics in safety barrier certificates.

    This function accepts single-integrator dynamics and wraps them in barrier
    certificates to ensure that collisions do not occur. Note that this
    algorithm bounds the magnitude of the generated output to 0.1.

    Parameters
    ----------
    dxi : SOMETHING
        Single-integrator dynamics.
    x : SOMETHING
        States of the agents
    safety_radius : SOMETHING
        Size of the agents (or desired separation distance).

    Returns
    -------
    dx : SOMETHING
        Generated safe, single-integrator inputs.

    """
    n = dxi.shape[1]
    gamma = 1e4
    x = x[0:2, :]

    a_mat = np.array()
    b_mat = np.array()

    for i in range(0, n-1):
        for j in range(i+1, n):
            h = np.linalg.norm(np.power(x[0:2, i] - x[0:2, j], 2) -
                               np.power(safety_radius, 2))
            a_new = np.zeros((1, 2*n))
            a_new[0, (2*i-1):(2*i)] = -2 * np.transpose(x[:, i] - x[:, j])
            a_new[0, (2*j-1):(2*j)] = 2 * np.transpose(x[:, i] - x[:, j])
            a_mat = np.concatenate((a_mat, a_new), axis=0)
            b_mat = np.concatenate((b_mat, gamma * np.power(h, 3)), axis=0)

    v_hat = np.reshape(dxi, (2*n, 1))
    k_relax = 100
    h_cap = 2 * np.eye(2*n)
    f = -2 * v_hat
    relaxation_var = k_relax * 0.1 * np.ones((1, 2*n))
    v_new = quadprog()

    # Set robot velocities to new velocities
    dx = np.reshape(v_new, (2, n))
    return dx
