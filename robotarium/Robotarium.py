import numpy as np
import matplotlib.pyplot as plt
from matplotlib.text import Annotation
from matplotlib import animation


class Robotarium(object):
    """ Simulator Interface for the Robotarium.

        NOTE: A position controller is not applied by default, therefore it must
        be added through the set_position_controller() method.

        Parameters
        ----------
        ion : bool (Default=True)
            Bool for displaying the matplotlib plot for dynamic animation. THIS
            SHOULD NOT BE CHANGED unless you just want a quick visualization of
            agent placement.
        anim_speed : float (Default=0.05)
            The speed at which matplotlib will update animations.
        low_quality : bool (Default=False)
            This mode reduces the number of points used to generate the plt.Polygon
            object, which allows for faster rendering of behaviors. Use this mode
            for slower computers or very large numbers of agents.

        Attributes
        ----------
        time_step : float
            The increment of time for calculating velocity.
        figure_handle : matplotlib figure object
            This handle contains the figure for which the grits bots will be
            displayed.

        Methods
        -------
        initialize(n)
            Initialize the state of 'n' robots and start visualization.
        step()
            Update the state of agents and display.
        time_to_iters(time)
            Returns the number of iterations from the current time.

        set_position_controller(controller)
            Sets the position controller to use the simulation.
        set_velocities(ids, vs)
            Sets the velocities of the current agents.
        set_positions(ids, ps)
            Sets the velocities for the agents via the position controller.
        set_save_parameters(file_path, length, every)
            Sets the state saving parameters for the simulation.
        set_robot_color(new_color)
            Sets a matplotlib compliant color for the robot.

        get_d_disk_neighbors(ids, r)
            Gets the neighbors of a particular agent within r distance in the 2
            norm.
        get_top_neighbors(ids, laplacian)
            Gets teh topological neighbors of an agent, given a graph Laplacian.
        get_poses()
            Gets the (x, y, theta) poses of the robots.
        get_available_agents()
            Returns the number of available agents.
    """

    def __init__(self, ion=True, anim_speed=0.05, low_quality=False):
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
        self.__time = 0
        self.log = []

        # Dynamics and Parameters
        self.__num_agents = 4
        self.__linear_velocity_coef = 1
        self.__angular_velocity_coef = 1
        self.__max_linear_velocity = 0.1
        self.__max_angular_velocity = 2 * np.pi

        # Controllers (functions)
        self.__position_controller = None

        # Visualization
        self.__robot_handle = {}
        self.__boundaries = [-0.6, 0.6, -0.35, 0.35]
        self.__boundary_points = [[-0.6, 0.6, 0.6, -0.6],
                                  [-0.35, -0.35, 0.35, 0.35]]
        self.__robot_body = None
        self.__offset = 0.05
        self.__ion = ion
        self.__anim_speed = anim_speed
        self.__robot_number = {}
        self.__pos_bias = 0.01  # Bias for the annotation position.
        self.__robot_color = '#9521F6'
        self.__low_quality = low_quality

        # Barrier Certificates
        self.__gamma = 1e4
        self.__safety_radius = 0.1
        self.__diff_morphism_gain = 0.05

    def initialize(self, n):
        """ Initialize the state of 'n' robots and start visualization.

            Parameters
            ----------
            n : int
                Number of agents desired.
        """
        self.__num_agents = n
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
        """ Sets the position controller to use the simulation.

            Parameters
            ----------
            controller : obj
                Position (go-to-goal) controller.
        """

        self.__position_controller = controller

    def set_velocities(self, ids, vs):
        """ Sets the velocities of the current agents.

            Parameters
            ----------
            ids : array of int
                Identities of agents whose velocities to set.
            vs : array of ints
                Velocities to set.
        """
        n = vs.shape[1]

        for i in range(0, n):
            if np.absolute(vs[0, i]) > self.__max_linear_velocity:
                vs[0, i] = self.__max_linear_velocity * np.sign(vs[0, i])

            if np.absolute(vs[1, i]) > self.__max_angular_velocity:
                vs[1, i] = self.__max_angular_velocity * np.sign(vs[1, i])

        # Change the state of agents within the ids array
        self.__states[3:5, ids] = vs

    def set_positions(self, ids, ps):
        """ Sets the velocities for the agents via the position controller.

            Parameters
            ----------
            ids : array of int
                Identities of agent positions to set.
            ps : matrix of ints
                Goal positions of agents 2xN.
        """
        if self.__position_controller is None:
            print("ERROR: No position controller set. Use ",
                  "set_position_controller() to set one.")

        else:
            self.set_velocities(ids, self.__position_controller(
                self.__states[:, ids], ps))

    def set_save_parameters(self, file_path, length, every):
        """ Sets the state saving parameters for the simulation.

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

    def set_robot_color(self, new_color):
        """ Sets the color of the grits bots rendered on the plot.

            Parameters
            ----------
            new_color : str
                These are matplotlib standard color schemes and can take the
                following types
                    eg. new_color = 'w'  # White
                    eg. new_color = '#eeefff'  # Hex string color
                    eg. new_color = '0.75'  # Gray-scale color (0 - 1)

        """
        self.__robot_color = new_color

    def get_d_disk_neighbors(self, ids, rad):
        """ Gets the neighbors of a particular agent within r distance in the 2 norm.

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
            neighbors = neighbors[0, :]

        return neighbors

    def get_top_neighbors(self, ids, laplacian):
        """ Gets the topological neighbors of an agent, given a graph Laplacian.

            Parameters
            ----------
            ids : array of int
                Identity of agents whose neighbors to get.
            laplacian : array of int
                Graph Laplacian of the communication topology.
        """
        neighbors = np.zeros((1, self.__num_agents), dtype=np.int)
        count = 0

        for i in range(0, self.__num_agents):
            if (i is not ids) and (laplacian[ids, i] is not 0):
                count += 1
                neighbors[:, count] = i

        if count == 0:
            neighbors = []

        else:
            neighbors = neighbors[0, :]

        return neighbors

    def get_poses(self):
        """ Gets the (x, y, theta) poses of the robots. """
        return self.__states[0:3, :]

    def get_available_agents(self):
        """ Return the number of available agents. """
        return self.__num_agents

    def step(self):
        """ Update the state of agents.

            Using an agents velocities, the X, Y, and theta of each agent is
            updated for eventual rendering by the __draw_robots() private method.
        """
        # Update velocities using unicycle dynamics
        for i in range(0, self.__num_agents):
            self.log.append([self.__time, i] + list(self.__states[:, i]))
            self.__states[0, i] += self.__linear_velocity_coef * \
                self.time_step * np.multiply(self.__states[3, i],
                                             np.cos(self.__states[2, i]))
            self.__states[1, i] += self.__linear_velocity_coef * \
                self.time_step * np.multiply(self.__states[3, i],
                                             np.sin(self.__states[2, i]))
            self.__states[2, i] += self.__angular_velocity_coef * \
                self.time_step * self.__states[4, i]

            # Ensure we're in the right range.
            self.__states[2, i] = np.arctan2(np.sin(self.__states[2, i]),
                                             np.cos(self.__states[2, i]))

        self.__time += self.time_step

        # self.__save()
        self.__draw_robots()

        # ADD PART ABOUT PAUSE AND PREVIOUS TIME STEP.

    def time_to_iters(self, time):
        """ Return the number of iterations from the current time.

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
        """ Animates the motion of agents.

            Updated state values are applied to the matplotlib Polygon objects
            within the __robot_handle dictionary. Annotations containing the
            agent's number are updated to reflect the agents motion.
        """

        def init():
            return self.__robot_handle

        def animate(i):
            """ Draw motion of robots. """
            for j in range(0, self.__num_agents):
                x = self.__states[0, j]
                y = self.__states[1, j]
                th = self.__states[2, j]
                pose_transformation_mat = np.array([
                    [np.cos(th), -1*np.sin(th), x],
                    [np.sin(th), np.cos(th), y],
                    [0, 0, 1]])
                robot_body_transformed = np.dot(self.__robot_body,
                                                pose_transformation_mat.T)
                # New robot location.
                self.__robot_handle[j].set_xy(robot_body_transformed[:, 0:2])

                # New Annotation location.
                self.__robot_number[j].remove()
                self.__robot_number[j] = Annotation(
                    str(j + 1),
                    xy=(x - self.__pos_bias, y - self.__pos_bias),
                    xytext=(x - self.__pos_bias, y - self.__pos_bias))
                self.ax.add_artist(self.__robot_number[j])

            return self.__robot_handle

        # Animate the movement
        anim = animation.FuncAnimation(
            self.figure_handle, animate, frames=1, interval=0, init_func=init,
            repeat=False, blit=False)
        plt.draw()
        plt.pause(self.__anim_speed)

    def __init_robot_visualize(self):
        """ Initialize visualization for robots.

            The body of each robot consists of different parts, namely:
                grits_bot_left_wheel  --> X, Y, Z points for left wheel
                grits_bot_right_wheel --> X, Y, Z points for right wheel
                grits_bot_tail_pin    --> X, Y, Z points for tail pin
                grits_bot_base        --> X, Y, Z points for robot base

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
                                   [(-1 * val), val, 1],
                                   [(-1 * val), (-1 * val), 1],
                                   [val, (-1 * val), 1]])
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
        grits_bot_tail_pin_angle = np.arange((np.pi * 8 / 9),
                                             (np.pi * 10 / 9),
                                             (np.pi / 18))[:, np.newaxis]

        grits_bot_tail_pin = np.vstack(
            [np.array([[(-1 * val), np.sin(grits_bot_tail_pin_angle[0]), 1]]),
             np.hstack([np.cos(grits_bot_tail_pin_angle),
                        np.sin(grits_bot_tail_pin_angle),
                        np.ones(grits_bot_tail_pin_angle.shape)]),
             np.hstack([0.95 * np.cos(grits_bot_tail_pin_angle[::-1]),
                        0.95 * np.sin(grits_bot_tail_pin_angle[::-1]),
                        np.ones(grits_bot_tail_pin_angle.shape)]),
             np.array([[(-1 * val),
                        (0.95 * np.sin(grits_bot_tail_pin_angle[0])), 1]])
             ])

        grits_bot_tail_pin[:, 0:2] = grits_bot_tail_pin[:, 0:2] * \
            robot_diameter

        """ Create Low Quality Version """
        grits_bot_low_quality = np.array([[(-1 * val), (-1 * val), 1],
                                          [val, (-1 * val) + val/2, 1],
                                          [val, val - val/2, 1],
                                          [(-1 * val), val, 1],
                                          [(-1 * val), (-1 * val), 1],
                                          [val, (-1 * val) + val/2, 1]])

        # Define common patch variables
        if not self.__low_quality:
            self.__robot_body = np.vstack([grits_bot_left_wheel,
                                           grits_bot_right_wheel,
                                           grits_bot_tail_pin,
                                           grits_bot_base])

        else:
            self.__robot_body = grits_bot_low_quality
            self.__robot_body[:, 0:2] = self.__robot_body[:, 0:2] * \
                robot_diameter

        # Create empty dictionary to place plt.Polygon objects.
        for i in range(0, num_robots):
            self.__robot_handle[i] = None

        # Set initial placement of agents on graph.
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

            # Visualize each robot.
            self.__robot_handle[j] = plt.Polygon(
                robot_body_transformed[:, 0:2])
            self.__robot_handle[j].set_fill(True)
            self.__robot_handle[j].set_visible(True)
            self.__robot_handle[j].set_color(self.__robot_color)
            self.ax.add_patch(self.__robot_handle[j])

            # Annotate each new robot.
            self.__robot_number[j] = Annotation(str(j+1), xy=(x-.01, y-.01),
                                                xytext=(x-.01, y-.01))
            self.ax.add_artist(self.__robot_number[j])

        # Show plot.
        if self.__ion:
            plt.ion()
        plt.show()


if __name__ == '__main__':
    # Get Robotarium object used to communicate with the robots/simulator
    r = Robotarium(ion=False)

    # Get the number of available agents from the Robotarium.  We don't need a
    # specific value for this algorithm
    N = r.get_available_agents()

    # Initialize the Robotarium object with the desired number of agents
    r.initialize(N)
