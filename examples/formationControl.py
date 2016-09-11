import numpy as np
from robotarium import Robotarium, transformations


# Get Robotarium object used to communicate with the robots/simulator
r = Robotarium()

# Get the number of available agents from the Robotarium. We don't need
# a specific value for this algorithm.
n = 6

# Initialize the Robotarium object with the desired number of agents.
r.initialize(n)

# Gains for the transformation from single-integrator to unicycle
# dynamics
linear_velocity_gain = 1
angular_velocity_gain = np.pi / 2
formation_control_gain = 4

# Select the number of iterations for the experiment. This value is
# arbitrary
iterations = 500

# Communication topology for desired formation. We need 2 * N - 3 = 9
# edges to ensure that the formation is rigid.
lap = np.array([
    [3, -1, 0, -1, 0, -1],
    [-1, 3, -1, 0, -1, 0],
    [0, -1, 3, -1, 0, -1],
    [-1, 0, -1, 3, -1, 0],
    [0, -1, 0, -1, 3, -1],
    [-1, 0, -1, 0, -1, 3]])

# The desired inter-agent distance for the formation.
d = 0.2

# Pre-compute diagonal values for the rectangular formation
d_diag = np.sqrt(np.power(2*d, 2) + np.power(d, 2))

# Weight matrix
weights = np.array([
    [0, d, 0, d, 0, d_diag],
    [d, 0, d, 0, d, 0],
    [0, d, 0, d_diag, 0, d],
    [d, 0, d_diag, 0, d, 0],
    [0, d, 0, d, 0, d],
    [d_diag, 0, d, 0, d, 0]])

# Initialize velocity vector for agents. Each agents expects a 2x1
# velocity vector containing the linear and angular velocity, respectively.
dx = np.zeros((2, n))

# Iterate for the previously specified number of iterations.
for _ in range(0, iterations):
    # Retrieve the most recent poses from the Robotarium. The time delay is
    # approximately 0.033 seconds.
    x = r.get_poses()

    # ALGORITHM
    # This section contains the actual algorithm for formation control!

    # Calculate single integrator control inputs using edge-energy consensus
    for i in range(0, n):
        # Initialize velocity to zero for each agent. this allows us to sum
        # over agent i's neighbors
        dx[:, [i]] = np.zeros((2, 1))

        # Get the topological neighbors of agent i form the communication
        # topology
        for j in r.get_top_neighbors(i, lap):
            # For each neighbor, calculate appropriate formation control
            # term and add it to the total velocity.

            # FORMATION CONTROL
            dx[:, [i]] += \
                formation_control_gain * \
                (np.power(np.linalg.norm(x[0:2, [i]] - x[0:2, [j]]), 2) -
                 np.power(weights[i, j], 2)) * \
                np.subtract(x[0:2, [j]], x[0:2, [i]])

            # END FORMATION CONTROL

    # END ALGORITHM
    # Transform the single-integrator dynamics to unicycle dynamics using a
    # provided utility.
    dx = transformations.int_to_uni2(dx, x, linear_velocity_gain,
                                     angular_velocity_gain)

    # Set velocities of agents 1 to n.
    r.set_velocities(range(0, n), dx)

    # Send the previously set velocities to the agents. This function must be
    # called.
    r.step()
