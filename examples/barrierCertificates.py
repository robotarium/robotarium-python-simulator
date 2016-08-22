import numpy as np
from Robotarium import Robotarium
from utilities import transformations
from utilities import controllers


# Get Robotarium object used to communicate with the robots/simulator.
r = Robotarium()

# Get the number of available agents from the Robotarium. We don't need a
# specific value from this algorithm.
n = r.get_available_agents()

# Number of iterations.
iterations = 20000

# Initialize the Robotarium object with the desired number of agents.
r.initialize(n)

# Initialize velocity vector for agents. Each agent expects a 2x1 velocity
# vector containing the linear and angular velocity, respectively.
dx = np.zeros((2, n))

xy_bound = np.array([-0.5, 0.5, -0.3, 0.3])
p_theta = (np.arange(1, 2*n, 2)) / (2 * n) * 2 * np.pi
p_circ = np.vstack(
    [np.hstack([xy_bound[1] * np.cos(p_theta), xy_bound[1] * np.cos(p_theta + np.pi)]),
     np.hstack([xy_bound[3] * np.sin(p_theta), xy_bound[3] * np.sin(p_theta + np.pi)])])
x_goal = p_circ[:, 0:n]
flag = 0  # Flag of task completion

# iterate for the previously specified number of iterations.
for _ in range(0, iterations):
    # Retrieve teh most recent poses from teh Robotarium. The time delay is
    # approximately 0.033 seconds.
    x = r.get_poses()
    x_temp = x[0:2, :]

    # ALGORITHM

    # Nominal controller, go2goal
    if np.linalg.norm(x_goal-x_temp, ord=1) < 0.08:
        flag = 1 - flag

    if flag == 0:
        x_goal = p_circ[:, 0:n]

    else:
        x_goal = p_circ[:, n:2*n]

    # Use different go-to-goal
    dx = controllers.positionInt(x, x_goal, 0.05)

    # Saturation of controls
    dx_max = 0.1
    for i in range(0, n):
        if np.linalg.norm(dx[:, i]) > dx_max:
            dx[:, i] = dx[:, i] / np.linalg.norm(dx[:, i]) * dx_max

    # END ALGORITHM

    # Ensure the robots don't collide
    dx = transformations.barrierCertificate(dx, x, ds=0.1)

    # Transform the single-integrator dynamics to unicycle dynamics using a
    # diffeomorphism, which can be found in the utilities.
    dx = transformations.int_to_uni2(dx, x, 0.75, np.pi)

    # Set velocities of agents 1,...,n
    r.set_velocities(range(0, n), dx)

    # Send the previously set velocities to the agents.
    # This function must be called.
    r.step()
