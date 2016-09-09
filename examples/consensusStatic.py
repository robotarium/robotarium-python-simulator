import numpy as np
from robotarium import Robotarium, transformations, graph
import json

# Get Robotarium object used to communicate with the robots/simulator
r = Robotarium()

# Get the number of available agents from the Robotarium. We don't need a
# specific value for this algorithm.
n = r.get_available_agents()

# Initialize the Robotarium object with the desired number of agents.
r.initialize(n)

# Generate a cyclic graph Laplacian from our handy utilities. For this
# algorithm, any connected graph will yield consensus.
lap = graph.cycle_gl(n)

# Gain for the diffeomorphism transformation between single-integrator and
# unicycle dynamics.
diffeomorphism_gain = 0.25

# Select the number of iterations for the experiment. This value is arbitrary.
iterations = 1000

# Initialize velocity vector for agents. Each agent expects a 2x1 velocity
# vector containing the linear and angular velocity, respectively.
dx = np.zeros((2, n))

# Iterate for the previously specified number of iterations
for _ in range(0, iterations):
    # Retrieve the most recent poses from the Robotarium. This time delay is
    # approximately 0.033 seconds.
    x = r.get_poses()
    x_ = x[0, :]
    y_ = x[1, :]

    # ALGORITHM
    for i in range(0, n):
        # Initialize velocity to zero for each agent. This allows us to sum
        # over agent i's neighbors
        dx[:, [i]] = np.zeros((2, 1))

        # Get the topological neighbors of agent i based on the graph.
        neighbors = r.get_top_neighbors(i, lap)

        # Iterate through agent i's neighbors
        for j in neighbors:
            # For each neighbor, calculate appropriate consensus term and
            # add it to the total velocity.
            dx[:, i] += (x[0:2, j] - x[0:2, i])

    # END ALGORITHM
    dx = transformations.barrier_certificate(dx, x, ds=0.1)
    dx = transformations.int_to_uni3(dx, x, diffeomorphism_gain)

    # Set velocities of agents 1,...,n
    r.set_velocities(range(0, n), dx)

    # Send the previously set velocities to the agents.
    # This function must be called.
    r.step()

with open('log.json', 'w') as f:
    json.dump(r.log, f)
