# robotarium-python-simulator
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
