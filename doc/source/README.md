robotarium-python-simulator
===========================

This is a Python simulator for Robotarium! The Robotarium is a project started by Georgia Tech Professor Magnus Egerstedt allowing public, remote access to a state-of-the-art multi-robot testbed.

This project, however, is NOT associated with the Robotarium. This is an open-source, re-implementation of the [MATLAB simulator]  (https://github.com/robotarium/robotarium-matlab-simulator) in Python (provided by Magnus Egerstedt, Daniel Pickem, and pglotfel). The purpose of this project is to allow people to further experiment with the Robotarium simulator in the following ways:

1. Unable to have access to MATLAB.
2. Want the flexibility of Python.
3. Educational purposes, such as learning about multi-agent systems.

NOTE: A Script/Code generated with this simulator is not usable with the remote client set up by Magnus Egerstedt. This is ONLY for simulation purposes. To have a physical implementation code run, you must re-implement your code into MATLAB. The way this simulator is written should allow for painless re-implementation.

Credit is given to Magnus Egerstedt and Daniel Pickem for the [code] (https://github.com/robotarium/robotarium-matlab-simulator) from which this was derived (MIT License).

This work is done in association with the Autonomous Collective Systems Lab at Arizona State University.

## Installation

The simulator can run on all major platforms (Windows, Linux, and macOS). All that is required is cloning the repository and installing some necessary dependencies.

```
git clone https://github.com/zmk5/robotarium-python-simulator.git
```

The following dependencies are required for utilization of the simulator:
- [NumPy] (http://www.numpy.org)
- [matplotlib] (http://matplotlib.org/index.html)
- [CVXOPT] (http://cvxopt.org/index.html)

NOTE: The SciPy stack and matplotlib can be difficult to install on Windows. However, [this] (http://www.lfd.uci.edu/~gohlke/pythonlibs/) link provides .whl files necessary for installation. Make sure to install all the dependencies for each version part of the SciPy and matplotlib stack!

## Usage
To utilize the simulator, the following libraries will need to be imported:

```
import Robotarium

# If you need specific utlities, import the following:
from utilities import controllers
from utilities import graph
from utilities import misc
from utilities import transformation
```

Once the files are imported, the Robotarium object must be instantantiated:
 
 ```
 r = Robotarium()
 ```
 
 This call will initialize default values that may be changed after this point. Once complete, the creation of the robots is done through the following method call:
 
 ```
 r.initialize(n)
 ```
 
 where 'n' is a number of grits bots desired. Once these lines are called, a user may begin formulating algorithms for use.

## Python to MATLAB
Currently, the Robotarium exclusively uses MATLAB for running user scripts for remote-connection sessions. This means that scripts generated using this code will not work with the remote session tools. The code will need to be rewritten into MATLAB syntax to work with the Robotarium system. Another issue is that the code contained within this repository was made PEP8 compliant. Therefore, method and function calls need to be converted to their equivalent MATLAB calls. These changes are shown below:

```
 # Python Version                  # MATLAB Version
 
      # --- For the Robotarium class --- #
 # Methods
 r.initialize()               ---> r.initialize()
 r.step()                     ---> r.step()
 
 # Mutators
 r.set_position_controller()  ---> r.setPositionController()
 r.set_velocities()           ---> r.setVelocities()
 r.set_positions()            ---> r.setPositions()
 r.set_save_parameters()      ---> r.setSaveParameters()
 
 # Accessors
 r.get_d_disk_neighbors()     ---> r.getDDiskNeighbors()
 r.get_top_neighbors()        ---> r.getTopNeighbors()
 r.get_poses()                ---> r.getPoses()
 r.get_available_agents()     ---> r.getAvailableAgents()
 
      # --- utilities/controller folder --- #
 park()                       ---> park()
 position_clf()               ---> positionCLF()
 position_int()               ---> positionInt()
 
      # --- utilities/graph folder --- #
 complete_gl()                ---> completeGL()
 cycle_gl()                   ---> cycleGL()
 line_gl()                    ---> lineGL()
 random_connected_gl()        ---> randomConnectedGL()
 random_gl()                  ---> randomGL()
 
      # --- utilities/misc folder --- #
 is_initialized()             ---> isInitialized()
 unique_filename()            ---> uniqueFilename()
 
      # --- utilities/transformations folder --- #
 int_to_uni()                 ---> int2uni()
 int_to_uni2()                ---> int2uni2()
 int_to_uni3()                ---> int2uni3()
 uni_to_int()                 ---> uni2int()
 
```

Methods or functions not in this chart are not located within MATLAB version of this simulator.
