import numpy as np


def barrierUnicycle(dxu, x, safety_radius, lambda_val):
    """

    Parameters
    ----------
    dxu : SOMETHING
        Single-integrator dynamics.
    x :  SOMETHING
        States of the agent.
    safety_radius : int
        Size of the agents (or desired separation distance).
    lambda_val : SOMETHING
        SOMETHING

    Returns
    -------
    dx : SOMETHING
        Generated safe, single-integrator inputs.

    """
    n = dxu.shape[1]
    gamma = 1e4

    # Shift to single integrator
    xi = x[0:2, :] + lambda_val * np.vstack((np.cos(x[2, :]), np.sin(x[2, :])))
    dxi = uni2int(dxu, x, lambda_val)

    # Determine dimension of constraints
    count = 0
    for i in range(0, n-1):
        for j in range(i+1, n):
            count += 1

    # Generate constraints for barrier certificates based on the size of
    # the safety radius.
    a_mat = np.zeros((count, 2*n))
    b_mat = np.zeros((count, 1))
    for i in range(0, n-1):
        for j in range(i+1, n):
            h = np.power(np.linalg.norm(xi[0:2, i] - xi[0:2, j]), 2) - \
                np.power(safety_radius + 2 * lambda_val, 2)
            a_new = np.zeros(1, 2 * n)
            a_new[0, (2*i-1):(2*i)] = -2 * np.transpose(xi[:, i] - xi[:, j])
            a_new[0, (2*j-1):(2*j)] = 2 * np.transpose(xi[:, i] - xi[:, j])
            a_mat = np.concatenate((a_mat, a_new), axis=0)
            b_mat = np.concatenate((b_mat, gamma * np.power(h, 3)), axis=0)

    # Solve QP program generated earlier
    v_hat = np.reshape(dxi, (2*n, 1))
    k_relax = 1
    h_cap = 2 * np.eye(2*n)
    f = -2 * v_hat
    relaxation_var = k_relax * 1 * np.ones((1, 2*n))
    v_new = quadprog()

    # Set robot velocities to new velocities
    dx = int2uni(np.reshape(v_new, (2, n)), x, lambda_val)
    return dx
