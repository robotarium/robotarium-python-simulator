import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


class Robotarium(object):
    """
    Robotarium Simulator Interface for the Robotarium

    Attributes
    ----------
    time_step : SOMETHING
    figure_handle :

    Methods
    -------
    initialize(n)
        Initialize the state of 'n' robots and start visualization.
    set_position_controller(controller)
        Sets the position controller to use the simulation.
    set_velocities(ids, vs)
        Sets the velocities of the current agents.
    set_positions(ids, ps)
        Sets the velocities for the agents via the position controller.
    set_save_parameters(file_path, length, every)
        Sets the state saving parameters for the simulation.
    get_d_disk_neighbors(ids, r)
        Gets the neighbors of a particular agent within r distance in the 2
        norm.
    get_top_neighbors(ids, laplacian)
        Gets teh topological neighbors of an agent, given a graph Laplacian.
    get_poses()
        Gets the (x, y, theta) poses of the robots.
    get_available_agents()
        Returns the number of available agents.
    step()
        SOMETHING
    time_to_iters(time)
        Returns the number of iterations from the current time.

    """

    def __init__(self):
        # --- Public Attributes --- #
        self.time_step = 0.033
        self.figure_handle = None

        # --- Private Attributes --- #
        # Data Logging Parameters
        self.__states = None
        self.__temp_states = []
        self.__save_length = 0
        self.__save_every = 0
        self.__prev_iters = 1
        self.__iters = 1
        self.__file_path = None
        self.__previous_time_step = None

        # Dynamics and Parameters
        self.__num_agents = 4
        self.__linear_velocity_coef = 1
        self.__angular_velocity_coef = 1
        self.__max_linear_velocity = 0.1
        self.__max_angular_velocity = 2 * np.pi

        # Controllers (functions)
        self.__position_controller = None

        # visualization
        self.__robot_handle = {}
        self.__boundaries = [-0.6, 0.6, -0.35, 0.35]
        self.__boundary_points = [[-0.6, 0.6, 0.6, -0.6],
                                  [-0.35, -0.35, 0.35, 0.35]]
        self.__robot_body = None
        self.__offset = 0.05

        # Barrier Certificates
        self.__gamma = 1e4
        self.__safety_radius = 0.1
        self.diffeomorphism_gain = 0.05

    def initialize(self, n):
        """
        Initialize the state of 'n' robots and start visualization.

        Parameters
        ----------
        n : int
            Number of agents desired.

        """
        self.__states = np.zeros((5, n))

        num_x = int(np.floor(1.2 / self.__safety_radius))
        num_y = int(np.floor(0.7 / self.__safety_radius))
        values = np.random.permutation(num_x * num_y)
        values = np.resize(values, (1, n))

        # Create a random X, Y, and Theta start position.
        for i in range(0, n):
            x, y = np.unravel_index(values[0, i], (num_x, num_y))
            x = x * self.__safety_radius - 0.6
            y = y * self.__safety_radius - 0.35
            self.__states[0, i] = x
            self.__states[1, i] = y
            self.__states[2, i] = np.random.rand() * 2 * np.pi

        # Start Visualization
        self.__init_robot_visualize()

    def set_position_controller(self, controller):
        """
        Sets the position controller to use the simulation.

        Parameters
        ----------
        controller : obj
            Position (go-to-goal) controller.

        """
        self.__position_controller = controller

    def set_velocities(self, ids, vs):
        """
        Sets the velocities of the current agents.

        Parameters
        ----------
        ids : array of int
            Identities of agents whose velocities to set.
        vs : array of ints
            Velocities to set.

        """
        a, n = vs.shape

        for i in range(0, n):
            if np.absolute(vs[0, i]) > self.__max_linear_velocity:
                vs[0, i] = self.__max_linear_velocity * np.sign(vs[0, i])

            if np.absolute(vs[1, i]) > self.__max_angular_velocity:
                vs[1, i] = self.__max_angular_velocity * np.sign(vs[1, i])

        # NEED TO CHECK WHAT IDS IS.
        self.__states[4:6, ids] = vs

    def set_positions(self, ids, ps):
        """
        Sets the velocities for the agents via the position controller.

        Parameters
        ----------
        ids : array of int
            Identities of agent positions to set.
        ps : matrix of ints
            Goal positions of agents 2 x N.

        """
        self.set_velocities(ids,
                            self.__position_controller(self.__states[:, ids],
                                                       ps))

    def set_save_parameters(self, file_path, length, every):
        """
        Sets the state saving parameters for the simulation.

        Parameters
        ----------
        file_path : str
            Path to save file.
        length : int
            Number of state iterations to save.
        every : int
            Save per 'every' iterations.

        """
        self.__save_length = length
        self.__save_every = every
        self.__temp_states = np.zeros((5 * self.__num_agents, every))
        self.__file_path = file_path
        # robot_states = np.zeros((5 * self.__num_agents, length))

        # Save to an external file.

    def get_d_disk_neighbors(self, ids, rad):
        """
        Gets the neighbors of a particular agent within r distance in the
        2 norm.

        Parameters
        ----------
        ids : int
            Identity of agent whose neighbors to get_poses.
        rad : int
            Radius of delta disk.

        """
        neighbors = np.zeros((1, self.__num_agents))
        count = 0

        for i in range(0, self.__num_agents):
            if (i is not id) and (np.linalg.norm(self.__states[0:2, ids] -
                                                 self.__states[0:2, i]) < rad):
                count += 1
                neighbors[:, count] = i

        if count == 0:
            neighbors = []

        else:
            neighbors = np.resize(neighbors, (1, count))

        return neighbors

    def get_top_neighbors(self, ids, laplacian):
        """
        Gets the topological neighbors of an agent, given a graph Laplacian.

        Parameters
        ----------
        ids : array of int
            Identity of agents whose neighbors to get.
        laplacian : array of int
            Graph Laplacian of the communication topology.

        """
        neighbors = np.zeros((1, self.__num_agents))
        count = 0

        for i in range(0, self.__num_agents):
            if (i is not ids) and (laplacian[ids, i] is not 0):
                count += 1
                neighbors[:, count] = i

        if count == 0:
            neighbors = []

        else:
            neighbors = np.resize(neighbors, (1, count))

        return neighbors

    def get_poses(self):
        """ Gets the (x, y, theta) poses of the robots. """
        return self.__states[0:3, :]

    def get_available_agents(self):
        """ Return the number of available agents. """
        return self.__num_agents

    def step(self):
        """ ADD SOMETHING. """
        # Update velocities using unicycle dynamics
        for i in range(0, self.__num_agents):
            self.__states[0, i] += self.__linear_velocity_coef * \
                self.time_step * np.multiply(self.__states[3, i],
                                             np.cos(self.__states[2, i]))
            self.__states[1, i] = self.__linear_velocity_coef * \
                self.time_step * np.multiply(self.__states[3, i],
                                             np.sin(self.__states[2, i]))
            self.__states[2, i] = self.__angular_velocity_coef * \
                self.time_step * self.__states[4, i]

            # Ensure we're in the right range.
            self.__states[2, i] = np.arctan2(np.sin(self.__states[2, i]),
                                             np.cos(self.__states[2, i]))

        self.__save()
        self.__draw_robots()
        # ADD PART ABOUT PAUSE AND PREVIOUS TIME STEP.

    def time_to_iters(self, time):
        """
        Return the number of iterations from the current time.

        Parameters
        ----------
        time : SOMETHING
            ADD SOMETHING

        """
        return np.ceil(time / self.time_step)

    def __save(self):
        """ Save data. NEED TO FIGURE OUT A WAY To SAVE THIS DATA. """
        if np.all(self.__temp_states):
            if self.__prev_iters <= self.__save_length:
                # Write to mat-file.
                pass

                if (self.__iters - 1) == self.__save_every:
                    pass

            else:
                pass

    def __draw_robots(self):
        """ Draw motion of robots. """
        for i in range(0, self.__num_agents):
            x = self.__states[0, i]
            y = self.__states[1, i]
            th = self.__states[2, i]
            pose_transformation_mat = np.array([
                [np.cos(th), -1*np.sin(th), x],
                [np.sin(th), np.cos(th), y],
                [0, 0, 1]])
            robot_body_transformed = np.dot(self.__robot_body,
                                            pose_transformation_mat.T)
            print(robot_body_transformed)

    def __init_robot_visualize(self):
        """ Initialize visualization for robots.

        The body of each robot consists of different parts, namely:
            grits_bot_left_wheel  --> X, Y, Z points for left wheel
            grits_bot_right_wheel --> X, Y, Z points for right wheel
            grits_bot_tail_pin    --> X, Y, Z points for tail pin
            grits_bot_base        --> X, Y, Z points for robot base
            grits_bot_tag         --> X, Y, Z points for april tag

        Each array consists for points on the graph that MATLAB/matplotlib
        use to form shapes. For example:
            grits_bot_base = [ 0, 0, 1;
                               1, 0, 1;
                               1, 1, 1;
                               0, 1, 1];

        When all points are connected by lines, the grits bot base is
        generated.

        All robots are stored in the __robot_handle dictionary. Each robot
        is a matplotlib Polygon object and can be accessed with the
        appropriate index value.

        """
        # Initialize variables.
        robot_diameter = 0.03
        num_robots = self.__num_agents

        # Scale Factor (max value of single gaussian)
        # scale_factor = 0.5
        fig_phi = plt.figure(1)
        self.figure_handle = fig_phi
        self.ax = self.figure_handle.add_subplot(111)

        # Plot Robotarium boundaries
        self.ax.spines['top'].set_color('c')
        self.ax.spines['bottom'].set_color('c')
        self.ax.spines['left'].set_color('c')
        self.ax.spines['right'].set_color('c')
        self.ax.set_xlim([self.__boundaries[0] - self.__offset,
                          self.__boundaries[1] + self.__offset])
        self.ax.xaxis.set_visible(False)
        self.ax.set_ylim([self.__boundaries[2] - self.__offset,
                          self.__boundaries[3] + self.__offset])
        self.ax.yaxis.set_visible(False)

        # Define custom patch variables
        """ Creating Grits Bot Base Matrix. """
        val = np.sqrt(2) / 2

        grits_bot_base = np.array([[(-1 * val), (-1 * val), 1],
                                   [val, (-1 * val), 1],
                                   [val, val, 1],
                                   [(-1 * val), val, 1]])
        grits_bot_base[:, 0:2] = grits_bot_base[:, 0:2] * robot_diameter

        """ Creating Grits Bot Left and Right Wheel Matrix """
        grits_bot_wheel = np.array([[(-1 * val), (-1 * val), 1],
                                    [val, (-1 * val), 1],
                                    [val, val, 1],
                                    [(-1 * val), val, 1]])
        grits_bot_wheel[:, 0:2] = np.dot(grits_bot_wheel[:, 0:2],
                                         np.diag([(robot_diameter / 3),
                                                 (robot_diameter / 6)]))
        grits_bot_left_wheel = grits_bot_wheel + \
            np.array([0, (7 / 6 * val * robot_diameter), 0])
        grits_bot_right_wheel = grits_bot_wheel + \
            np.array([0, (-7 / 6 * val * robot_diameter), 0])

        """ Creating Grits Bot Tail Pin Matrix """
        grits_bot_tail_pin_angle = np.arange((np.pi * 8 / 9), (np.pi * 10 / 9),
                                             (np.pi / 18))[:, np.newaxis]

        a = np.array([[(-1*val), np.sin(grits_bot_tail_pin_angle[0]), 1]])

        b_1 = np.concatenate((np.cos(grits_bot_tail_pin_angle),
                              np.sin(grits_bot_tail_pin_angle)), axis=1)
        b_2 = np.concatenate((b_1, np.ones(grits_bot_tail_pin_angle.shape)),
                             axis=1)
        b = np.concatenate((a, b_2), axis=0)

        c_1 = np.concatenate((0.95 * np.cos(grits_bot_tail_pin_angle[::-1]),
                              0.95 * np.sin(grits_bot_tail_pin_angle[::-1])),
                             axis=1)
        c_2 = np.concatenate((c_1, np.ones(grits_bot_tail_pin_angle.shape)),
                             axis=1)
        c = np.concatenate((b, c_2), axis=0)

        d_1 = np.array([[(-1*val),
                         (0.95 * np.sin(grits_bot_tail_pin_angle[0])), 1]])
        grits_bot_tail_pin = np.concatenate((c, d_1), axis=0)
        grits_bot_tail_pin[:, 0:2] = grits_bot_tail_pin[:, 0:2] * \
            robot_diameter

        # --- FOR DEBUGGING --- #
        # print('grits_bot_base: \n', grits_bot_base)
        # print('grits_bot_base[:, 0:2]: \n', grits_bot_base[:, 0:2])
        # print('grits_bot_wheel: \n', grits_bot_wheel)
        # print('grits_bot_wheel[:, 0:2]: \n', grits_bot_wheel[:, 0:2])
        # print('grits_bot_left_wheel: \n', grits_bot_left_wheel)
        # print('grits_bot_right_wheel: \n', grits_bot_right_wheel)
        # print('grits_bot_tail_pin_angle: \n', grits_bot_tail_pin_angle)
        # print('grits_bot_tail_pin: \n', grits_bot_tail_pin)
        # print('grits_bot_tail_pin[:, 0:2]: \n', grits_bot_tail_pin[:, 0:2])
        # print('grits_bot_base_color: ', grits_bot_base_color)
        # print('grits_bot_wheel_color: ', grits_bot_wheel_color)
        # print('grits_bot_tail_pin_color: ', grits_bot_tail_pin_color)
        # print('grits_bot_tag_white: ', grits_bot_tag_white)
        # print('grits_bot_tag_black: ', grits_bot_tag_black)

        """ Create Grits Bot Tag. """
        # Define a unit circle circumscribed rectangleBox
        rectangle_box = np.array([[(-1 * val), (-1 * val)],
                                  [val, (-1 * val)],
                                  [val, val],
                                  [(-1 * val), val]])
        aruco_tag_scale = 0.8
        rectangle_box = rectangle_box * aruco_tag_scale * robot_diameter

        aruco_box_scale = 0.15
        aruco_box_shift_scale = 1 * aruco_box_scale

        a_1 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[0, 0],
                                                3 * rectangle_box[0, 1]]]))
        a_2 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[0, 0],
                                                3 * rectangle_box[0, 1]]]))
        a_3 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[1, 0],
                                                3 * rectangle_box[0, 1]]]))
        a_4 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[1, 0],
                                                3 * rectangle_box[0, 1]]]))

        b_1 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[0, 0],
                                                rectangle_box[0, 1]]]))
        b_2 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[0, 0],
                                                rectangle_box[0, 1]]]))
        b_3 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[1, 0],
                                                rectangle_box[0, 1]]]))
        b_4 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[1, 0],
                                                rectangle_box[0, 1]]]))

        c_1 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[0, 0],
                                                rectangle_box[2, 1]]]))
        c_2 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[0, 0],
                                                rectangle_box[2, 1]]]))
        c_3 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[1, 0],
                                                rectangle_box[2, 1]]]))
        c_4 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[1, 0],
                                                rectangle_box[2, 1]]]))

        d_1 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[0, 0],
                                                3 * rectangle_box[2, 1]]]))
        d_2 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[0, 0],
                                                3 * rectangle_box[2, 1]]]))
        d_3 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[rectangle_box[1, 0],
                                                3 * rectangle_box[2, 1]]]))
        d_4 = (aruco_box_scale * rectangle_box) + \
            (aruco_box_shift_scale * np.array([[3 * rectangle_box[1, 0],
                                                3 * rectangle_box[2, 1]]]))

        outer_white_box = np.concatenate((rectangle_box, np.ones((4, 1))),
                                         axis=1)
        inner_black_box = np.concatenate((0.9*rectangle_box, np.ones((4, 1))),
                                         axis=1)
        grid_1_1 = np.concatenate((a_1, np.ones((4, 1))), axis=1)
        grid_1_2 = np.concatenate((a_2, np.ones((4, 1))), axis=1)
        grid_1_3 = np.concatenate((a_3, np.ones((4, 1))), axis=1)
        grid_1_4 = np.concatenate((a_4, np.ones((4, 1))), axis=1)

        grid_2_1 = np.concatenate((b_1, np.ones((4, 1))), axis=1)
        grid_2_2 = np.concatenate((b_2, np.ones((4, 1))), axis=1)
        grid_2_3 = np.concatenate((b_3, np.ones((4, 1))), axis=1)
        grid_2_4 = np.concatenate((b_4, np.ones((4, 1))), axis=1)

        grid_3_1 = np.concatenate((c_1, np.ones((4, 1))), axis=1)
        grid_3_2 = np.concatenate((c_2, np.ones((4, 1))), axis=1)
        grid_3_3 = np.concatenate((c_3, np.ones((4, 1))), axis=1)
        grid_3_4 = np.concatenate((c_4, np.ones((4, 1))), axis=1)

        grid_4_1 = np.concatenate((d_1, np.ones((4, 1))), axis=1)
        grid_4_2 = np.concatenate((d_2, np.ones((4, 1))), axis=1)
        grid_4_3 = np.concatenate((d_3, np.ones((4, 1))), axis=1)
        grid_4_4 = np.concatenate((d_4, np.ones((4, 1))), axis=1)

        grits_bot_tag = np.vstack((outer_white_box, inner_black_box,
                                   grid_1_1, grid_1_2, grid_1_3, grid_1_4,
                                   grid_2_1, grid_2_2, grid_2_3, grid_2_4,
                                   grid_3_1, grid_3_2, grid_3_3, grid_3_4,
                                   grid_4_1, grid_4_2, grid_4_3, grid_4_4))

        # Define common patch variables
        self.__robot_body = np.concatenate((grits_bot_left_wheel,
                                            grits_bot_right_wheel), axis=0)
        self.__robot_body = np.concatenate((self.__robot_body,
                                            grits_bot_tail_pin), axis=0)
        self.__robot_body = np.concatenate((self.__robot_body,
                                            grits_bot_base), axis=0)
        self.__robot_body = np.concatenate((self.__robot_body,
                                            grits_bot_tag), axis=0)

        wheel_length = grits_bot_wheel.shape[0]
        base_length = grits_bot_base.shape[0]
        tail_length = grits_bot_tail_pin.shape[0]
        a = np.empty(8-grits_bot_base.shape[0]+grits_bot_wheel.shape[0])
        b = np.empty(4-grits_bot_base.shape[0]+2*grits_bot_wheel.shape[0])
        c = np.empty(12-grits_bot_base.shape[0])
        d = np.empty((18, 8))
        a[:] = np.NaN
        b[:] = np.NaN
        c[:] = np.NaN
        d[:, :] = np.NaN
        temp = np.arange(1, (18*rectangle_box.shape[0]) + 1) + \
            (grits_bot_base.shape[0] + 2 * grits_bot_wheel.shape[0] +
                grits_bot_tail_pin.shape[0])

        row_1 = np.array([np.concatenate((
            np.arange(1, (grits_bot_wheel.shape[0]+1)), a), axis=0)])
        row_2 = np.array([np.concatenate((
            np.arange(1 + wheel_length, (2 * wheel_length) + 1), b), axis=0)])
        row_3 = np.array([np.arange(1 + 2 * wheel_length,
                          (2 * wheel_length + tail_length) + 1)])
        row_4 = np.array([np.concatenate((
            np.arange(1 + 2 * wheel_length + tail_length,
                      (2 * wheel_length + tail_length + base_length) + 1), c),
            axis=0)])
        row_5 = np.concatenate((np.reshape(temp, (18, 4)), d), axis=1)
        robot_faces = np.concatenate((row_1, row_2), axis=0)
        robot_faces = np.concatenate((robot_faces, row_3), axis=0)
        robot_faces = np.concatenate((robot_faces, row_4), axis=0)
        robot_faces = np.concatenate((robot_faces, row_5), axis=0)
        print(robot_faces)

        """ Color of individual patches. """
        grits_bot_base_color = '#9521F6'
        grits_bot_wheel_color = 'b'
        grits_bot_tail_pin_color = '#CCCCCC'
        grits_bot_tag_white = 'w'
        grits_bot_tag_black = 'b'

        robot_color = [grits_bot_wheel_color,
                       grits_bot_wheel_color,
                       grits_bot_tail_pin_color,
                       grits_bot_base_color,
                       grits_bot_tag_white,
                       grits_bot_tag_black]

        for i in range(0, num_robots):
            self.__robot_handle[i] = None

        for j in range(0, num_robots):
            x = self.__states[0, j]
            y = self.__states[1, j]
            th = self.__states[2, j]

            pose_transformation_mat = np.array([
                [np.cos(th), -1*np.sin(th), x],
                [np.sin(th), np.cos(th), y],
                [0, 0, 1]])

            robot_body_transformed = np.dot(self.__robot_body,
                                            pose_transformation_mat.T)

            self.__robot_handle[j] = plt.Polygon(
                robot_body_transformed[:, 0:2])
            self.__robot_handle[j].set_fill(True)
            self.__robot_handle[j].set_visible(True)
            self.__robot_handle[j].set_color(robot_color[3])
            self.ax.add_patch(self.__robot_handle[j])

        # Show plot.
        plt.show()

if __name__ == '__main__':
    # Get Robotarium object used to communicate with the robots/simulator
    r = Robotarium()

    # Get the number of available agents from the Robotarium.  We don't need a
    # specific value for this algorithm
    N = r.get_available_agents()

    # Initialize the Robotarium object with the desired number of agents
    r.initialize(N)
