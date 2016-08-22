import numpy as np
import cvxopt


def barrierCertificate(dxi, x, dx_max=0.1, ds=0.08):
    """ Wraps single-integrator dynamics in safety barrier certificates.

    This function accepts single-integrator dynamics and wraps them in barrier
    certificates to ensure that collisions do not occur. Note that this
    algorithm bounds the magnitude of the generated output to 0.1.

    Parameters
    ----------
    dxi : SOMETHING
        Single-integrator dynamics.
    x : SOMETHING
        States of the agents.
    dx_max : float
        SOMETHING
    ds : float
        Safety radius, size of the agents (or desired separation distance).

    Returns
    -------
    dx_out : SOMETHING
        Generated safe, single-integrator inputs.

    """
    cvxopt.solvers.options['show_progress'] = False
    n = dxi.shape[1]
    gamma = 1e4
    x = x[0:2, :]
    dx_out = dxi.copy()

    a_mat = np.empty((0, 2*n))
    b_mat = np.empty((0, 1))

    for i in range(0, n-1):
        for j in range(i+1, n):
            h_mat = np.linalg.norm(np.power(x[0:2, i] - x[0:2, j], 2) -
                                   np.power(ds, 2))
            a_new = np.zeros((1, 2*n))
            a_new[0, (2*i):(2*i+2)] = -2 * np.transpose(x[:, [i]] - x[:, [j]])
            a_new[0, (2*j):(2*j+2)] = 2 * np.transpose(x[:, [i]] - x[:, [j]])
            a_mat = np.vstack((a_mat, a_new))
            b_mat = np.vstack((b_mat, gamma * np.power(h_mat, 3)))

    # Add velocity bounds to all robots
    g_mat = np.vstack([a_mat, -1 * np.eye(2*n), np.eye(2*n)])
    h_mat = np.vstack([b_mat, dx_max * np.ones((2*n, 1)),
                       dx_max * np.ones((2*n, 1))])
    h_cap = 2 * np.eye(2*n)
    f_mat = -2 * np.reshape(np.transpose(dxi), (2*n, 1))

    # Solve LP to check for feasibility
    m_0, n_0 = a_mat.shape
    m, n = g_mat.shape
    aa = np.hstack([g_mat, -np.ones((m, 1))])
    bb = h_mat
    cc = np.zeros(n+1)
    cc[-1] = 1
    sol = cvxopt.solvers.lp(cvxopt.matrix(cc),
                            cvxopt.matrix(aa),
                            cvxopt.matrix(bb))
    x = sol['x']
    lp_tol = 0.05

    if x[-1] < 0:  # feasible
        sol = cvxopt.solvers.qp(cvxopt.matrix(h_cap),
                                cvxopt.matrix(f_mat),
                                cvxopt.matrix(g_mat),
                                cvxopt.matrix(h_mat),
                                cvxopt.matrix(np.empty((0, n))),
                                cvxopt.matrix(np.empty((0, 1))))
        x = sol['x']

    else:
        print('QP not feasible, LP result is used instead!')

    for i in range(0, int(n/2)):
        dx_out[:, i] = np.transpose(x[2*i:2*i+2])

    return dx_out


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
    dxi = uni_to_int(dxu, x, lambda_val)

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
    v_new = cvxopt.solvers.qp()

    # Set robot velocities to new velocities
    dx = int_to_uni(np.reshape(v_new, (2, n)), x, lambda_val)
    return dx


def int_to_uni(dxi, x, lambda_val):
    """
    Translates from single integrator to unicycle dynamics.

    Parameters
    ----------
    dxi : something
        SOMETHING
    x : something
        Unicycle states 3 x N
    lambda_val :


    Returns
    -------
    dx : SOMETHING
        SOMETHING

    """
    n = dxi.shape[1]
    dx = np.zeros((2, n))
    t = np.array([[1, 0], [0, 1/lambda_val]])

    for i in range(0, n):
        temp = np.array([[np.cos(x[2, i]), np.sin(x[2, i])],
                         [-1*np.sin(x[2, i]), np.cos(x[2, i])]])
        dx[:, i] = t * np.dot(temp, dxi[:, i])

    return dx


def int_to_uni2(dxi, states, kv, kw):
    """

    Parameters
    ----------
    dxi :
        Single-integrator dynamics.
    states :
        unicycle states (3 x N)
    kv :
        Linear velocity gain.
    kw :
        Rotational velocity gain. Due to normalization, the w value will be
        in the range -kw to kw.

    Returns
    -------
    dx

    """
    n = dxi.shape[1]
    dx = np.zeros((2, n))

    for i in range(0, n):
        temp_1 = np.array([[np.cos(states[2, i]), np.sin(states[2, i])]])
        temp_2 = np.array([[-1 * np.sin(states[2, i]), np.cos(states[2, i])]])
        dx[0, i] = kv * np.dot(temp_1, dxi[:, i])

        # Normalizing the output of atan2 to between -kw and kw
        dx[1, i] = kw * np.arctan2(np.dot(temp_2, dxi[:, i]),
                                   np.dot(temp_1, dxi[:, i])/(np.pi/2))

    return dx


def int_to_uni3(dxi, x, lambda_val):
    """
    Translates from single integrator to unicycle dynamics.

    Parameters
    ----------
    dxi :
        Single integrator control input.
    x :
        Unicycle states (3 x N)
    lambda_val :


    Returns
    -------
    dx :


    """
    n = dxi.shape[1]
    dx = np.zeros((2, n))
    t = np.array([[1, 0], [0, 1/lambda_val]])

    for i in range(0, n):
        temp = np.array([[np.cos(x[2, i]), np.sin(x[2, i])],
                         [-1 * np.sin(x[2, i]), np.cos(x[2, i])]])
        dx[:, i] = np.dot(t, np.dot(temp, dxi[:, i]))
        if dx[0, i] < 0:
            dx[1, i] *= -1

    return dx


def uni_to_int(dxu, x, lambda_val):
    """
    Translates from single integrator to unicycle dynamics.

    Parameters
    ----------
    dxu :
        Single integrator control input.
    x :
        Unicycle states (3 x N)
    lambda_val :


    Returns
    -------
    dx :


    """
    n = dxu.shape[1]
    dx = np.zeros((2, n))

    for i in range(0, n):
        temp = np.array([[np.cos(x[2, i]), -lambda_val * np.sin(x[2, i])],
                         [np.sin(x[2, i]), lambda_val * np.cos(x[2, i])]])
        dx[:, i] = np.dot(temp, dxu[:, i])

    return dx

