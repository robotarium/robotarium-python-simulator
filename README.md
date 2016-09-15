Introduction
============

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

## Dependency Installation

The guide below will show you how to install the necessary dependencies. Since the simulator can work on both Python 2.7.x and Python 3.5.x versions, the installation is given for both options. You do not need to install both!

### Linux
To install the simulator on linux requires the installation of the dependencies labeled above. The installation varies depending on the distribution used. The easiest way to install CVXOPT is to use pip, which is typically installed with the default python installation.

#### Ubuntu, Debian, and other Ubuntu/Debian based distributions.
```
# Python 2.7.x
sudo apt-get install python-numpy python-scipy python-matplotlib python-pip
pip install cvxopt --user

# Python 3.5.x
sudo apt-get install python3-numpy python3-scipy python3-matplotlib python3-pip
pip3 install cvxopt --user
```

#### Fedora, CentOS, and other RPM based distributions.
```
# Python 2.7.x
sudo yum install numpy scipy python-matplotlib python-pip  # For YUM package manager.
sudo dnf install numpy scipy python-matplotlib python-pip  # For DNF package manager.
pip install cvxopt --user

# Python 3.5.x
sudo yum install numpy scipy python3-matplotlib python3-pip  # For YUM package manager.
sudo dnf install numpy scipy python3-matplotlib python3-pip  # For DNF package manager.
pip3 install cvxopt --user
```

#### pip
If you are already using python with (or without) pip installed and configured, the installation can be done simply with the following commands:

```
# For Python 2.7.x
sudo apt-get install python-pip   # Ubuntu/Debian
sudo yum install python-pip   # Fedora/CentOS (RPM based YUM)
sudo dnf install python-pip   # Fedora/CentOS (RPM based DNF)

pip install scipy
pip install numpy
pip install matplotlib
pip install cvxopt --user

# For Python 3.5.x
sudo apt-get install python3-pip  # Ubuntu/Debian based
sudo yum install python3-pip  # Fedora/CentOS based (RPM Yum based)
sudo dnf install python3-pip  # Fedora/CentOS based (RPM dnf based)

pip3 install scipy
pip3 install numpy
pip3 install matplotlib
pip3 install cvxopt --user
```

### macOS
To install the simulator on macOS, it is recommended to install a package manager for easy installation. CVXOPT will have to be installed using PIP.

#### Homebrew
To use [Homebrew] (http://brew.sh) for dependency installation requires a bit of extra work due to the scipy stack not being a part of the main repository. You can then install the dependencies labeled above using the following work around (Requires PIP). A more detailed explanation can be found [here] (https://penandpants.com/2012/02/24/install-python/).

```
# Install Python (Choose Python 2.7.x or 3.5.x)
brew install python
brew install python3

# Restart terminal to allow the path to python to be updated.
# make sure "which python" command returns "/usr/local/bin/python"

# Install pip
easy_install pip

# Install NumPy
pip install numpy

# Install SciPy
brew install gfortran  # Install to prevent an error inherent in SciPy.
pip install scipy

# Install matplotlib
brew install pkg-config
pip install matplotlib

# Install CVXOPT
pip install cvxopt --user
```

#### Macports
To use [Macports] (https://www.macports.org/), use the following commands to install the scipy stack. At the time of writing, a Python 3.5.x version for the NumPy stack do not exist. 
```
# For Python 2.7+
sudo port install py27-numpy py27-scipy py27-matplotlib

# Install pip
easy_install pip

# Install CVXOPT
pip install cvxopt --user
```

### Windows
Of the three installations, this one will be the most difficult due to the fact that Windows does not come with a native or easily installable package manager. To circumvent these problems, it will be necessary to install the packages using pip. The issue with using pip, however, is that NumPy, SciPy, and matplotlib require the packages to be installed without compiling. Therefore, each wheel must be installed individually. This is a simple process using pip 8.x version. The following commands are for python installations that are using PIP 8.x version. The wheel files used here can be found [here] (http://www.lfd.uci.edu/~gohlke/pythonlibs/).

NOTE: The following files installed are for 64-bit architectures. If you have a 32-bit CPU, download the corresponding 32-bit and python versions of the files specified below.

#### Install NumPy
```
# Install NumPy (64-bit)
pip install numpy-1.11.1+mkl-cp27-cp27m-win_amd64.whl  # Python 2.7.x Version
pip install numpy-1.11.1+mkl-cp34-cp34m-win_amd64.whl  # Python 3.4.x Version
pip install numpy-1.11.1+mkl-cp35-cp35m-win_amd64.whl  # Python 3.5.x Version

# Install NumPy (32-bit)
pip install numpy-1.11.1+mkl-cp27-cp27m-win32.whl  # Python 2.7.x Version
pip install numpy-1.11.1+mkl-cp34-cp34m-win32.whl  # Python 3.4.x Version
pip install numpy-1.11.1+mkl-cp35-cp35m-win32.whl  # Python 3.5.x Version
```

#### Install SciPy
```
# Install SciPy (64-bit)
pip install scipy-0.18.0-cp27-cp27m-win_amd64.whl  # Python 2.7.x Version
pip install scipy-0.18.0-cp34-cp34m-win_amd64.whl  # Python 3.4.x Version
pip install scipy-0.18.0-cp35-cp35m-win_amd64.whl  # Python 3.5.x Version

# Install SciPy (32-bit)
pip install scipy-0.18.0-cp27-cp27m-win32.whl  # Python 2.7.x Version
pip install scipy-0.18.0-cp34-cp34m-win32.whl  # Python 3.4.x Version
pip install scipy-0.18.0-cp35-cp35m-win32.whl  # Python 3.5.x Version
```

#### Install matplotlib
Installation of matplotlib requires extra dependencies to be installed first.

```
# Install dateutil
pip install python_dateutil-2.5.3-py2.py3-none-any.whl

# Install pytz
pip install pytz-2016.6.1-py2.py3-none-any.whl

# Install pyparsing
pip install pyparsing-2.1.8-py2.py3-none-any.whl

# Install cycler
pip install cycler-0.10.0-py2.py3-none-any.whl

# Install setuptools
pip install setuptools-25.2.0-py2.py3-none-any.whl

# Install matplotlib (64-bit)
pip install matplotlib-1.5.2-cp27-cp27m-win_amd64.whl  # Python 2.7.x Version 
pip install matplotlib-1.5.2-cp34-cp34m-win_amd64.whl  # Python 3.4.x Version
pip install matplotlib-1.5.2-cp35-cp35m-win_amd64.whl  # Python 3.5.x Version

# Install matplotlib (32-bit)
pip install matplotlib-1.5.2-cp27-cp27m-win32.whl  # Python 2.7.x Version 
pip install matplotlib-1.5.2-cp34-cp34m-win32.whl  # Python 3.4.x Version
pip install matplotlib-1.5.2-cp35-cp35m-win32.whl  # Python 3.5.x Version
```

## Usage
To utilize the simulator, the following libraries will need to be imported:

```
from robotarium.Robotarium import Robotarium

# If you need specific utlities, import the following:
import robotarium.controllers
import robotarium.graph
import robotarium.transformation
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
 
 # --- utilities/transformations folder --- #
 int_to_uni()                 ---> int2uni()
 int_to_uni2()                ---> int2uni2()
 int_to_uni3()                ---> int2uni3()
 uni_to_int()                 ---> uni2int()
 
```

Methods or functions not in this chart are not located within MATLAB version of this simulator.
