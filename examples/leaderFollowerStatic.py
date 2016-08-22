import numpy as np
from Robotarium import Robotarium
from utilities import controllers
from utilities import graph
from utilities import transformations


if __name__ == '__main__':
    # Run for 300 Iterations
    iterations = 2000
    u_h = None

    # Get Robotarium object and set the save parameters
    r = Robotarium()
    n = r.get_available_agents()
    r.initialize(n)

    # Graph Laplacian
    followers = -1 * graph.completeGL(n-1)
    lap = np.zeros((n, n))
    lap[1:n, 1:n] = followers
    lap[1, 1] += 1
    lap[1, 0] = -1

    # Initialize velocity vector
    dx = np.zeros((2, n))

    # State for leader
    state = 0

    # Collision_avoidance_gain = 0.001
    formation_control_gain = 10
    desired_distance = 0.09

    for _ in range(0, 300):
        # Retrieve the most recent poses from the Robotarium. The time
        # delay is approximately 0.033 seconds.
        x = r.get_poses()

        # ALGORITHM
        for i in range(1, n):
            # Zero velocity and get the topological neighbors of agent i
            dx[:, [i]] = np.zeros((2, 1))
            neighbors = r.get_top_neighbors(i, lap)

            for j in neighbors:
                dx[:, i] += formation_control_gain * \
                            (np.power(np.linalg.norm(x[0:2, int(j)]
                                                     - x[0:2, i]), 2)
                             - np.power(desired_distance, 2)) * \
                            np.subtract(x[0:2, int(j)], x[0:2, i])

        # END ALGORITHM
        if state == 3:
            dx[:, [0]] = controllers.positionInt(x[:, 0],
                                                 np.array([[0.25], [-0.15]]),
                                                 0.05)
            if np.linalg.norm(x[0:2, 0] - np.array([[0.25], [-0.15]])) < 0.05:
                state = 4

        if state == 2:
            dx[:, [0]] = controllers.positionInt(x[:, 0],
                                                 np.array([[-0.25], [-0.15]]),
                                                 0.05)
            if np.linalg.norm(x[0:2, 0] - np.array([[-0.25], [-0.15]])) < 0.05:
                state = 3

        if state == 1:
            dx[:, [0]] = controllers.positionInt(x[:, 0],
                                                 np.array([[-0.25], [0.15]]),
                                                 0.05)
            if np.linalg.norm(x[0:2, 0] - np.array([[-0.25], [0.15]])) < 0.05:
                state = 2

        if state == 0:
            dx[:, [0]] = controllers.positionInt(x[:, [0]],
                                                 np.array([[0.25], [0.15]]),
                                                 0.05)
            if np.linalg.norm(x[0:2, 0] - np.array([[0.25], [0.15]])) < 0.05:
                state = 1

        dx[:, [0]] = (dx[:, [0]] / np.linalg.norm(dx[:, 0])) * 0.05
        dx = transformations.barrierCertificate(dx, x, ds=0.1)
        dx = transformations.int_to_uni2(dx, x, 1, 2)

        # Set velocities
        r.set_velocities(range(0, n), dx)

        # Iterate experiment
        r.step()
