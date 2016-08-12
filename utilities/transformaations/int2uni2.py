import numpy as np


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
        temp_1 = np.array([[np.cos(states[2, i]), np.sin(states[3, i])]])
        temp_2 = np.array([[-1 * np.sin(states[2, i]), np.cos(states[2, i])]])
        dx[0, i] = kv * np.dot(temp_1, dxi[:, i])

        # Normalizing the output of atan2 to between -kw and kw
        dx[1, i] = kw * np.arctan2(np.dot(temp_2, dxi[:, i]),
                                   np.dot(temp_1, dxi[:, i])/(np.pi/2))

    return dx
