# Doosan XBot2

This package contains the config files and instructions for running the Doosan robot with the XBot2 framework. It also contains a xbot2 example plugin to run a periodic motion on the robot.

At the moment, the package works with Ubuntu 20.04 and ROS Noetic.

## Dependencies

Install the following system dependencies:

- *ROS Noetic*
- *xbot2*: https://advrhumanoids.github.io/xbot2/master/index.html

Install the source dependencies (depending on the build tool you prefer):

- doosan-robot: https://github.com/doosan-robotics/doosan-robot

Let's first install its system dependencies (https://github.com/doosan-robotics/doosan-robot?tab=readme-ov-file#dependency-packages) and the source ones (e.g., serial), and then, if, for example, you are using catkin, it would be good to use the same workspace described below.

## How to run it?

### Setup and Configuration

Clone, compile, and install the repo (and its dependencies), for example, using a catkin package:


```
# create and initialize the catkin ws
mkdir -p magician_ws && cd magician_ws
mkdir -p src && catkin_init_workspace
 
# assuming you are using https
git clone https://github.com/magician-project/doosan_xbot2.git

# compile and install it
cd .. 
catkin_make install #or catkin config --install && catkin build
```

Source the catkin package if you are using this build tool and possibly include this source in the .bashrc:
```
. magician_ws/devel/setup.bash
```

Set the xbot2 configuration file as described in the tutorial here: https://github.com/ADVRHumanoids/xbot2_examples

```
set_xbot2_config doosan_xbot2/doosan_xbot2_config/doosan_xbot2_config.yaml
```

### Run the Kinematic Simulation

We offer an RViZ simulation of the Doosan robot, which you can run using the "dummy" control mode:  

**First Terminal**
```
. magician_ws/devel/setup.bash
roslaunch doosan_xbot2_config doosan_xbot2.launch 
```

**Second Terminal**
```
. magician_ws/install/setup.bash
xbot2-core --hw dummy
```

**Third Terminal**
We then offer the xbot2-gui to have a GUI to control and run plugins in the robot:
```
xbot2-gui
```

### Run the Dynamic Simulation on Gazebo

**First Terminal**
From the server side, run gazebo with the xbot2 doosan support:
```
. magician_ws/devel/setup.bash
roslaunch doosan_xbot2_gazebo doosan_xbot2_gazebo.launch 
```

**Second Terminal**
From the client side, run xbot2 in simulation mode:
```
. magician_ws/install/setup.bash
xbot2-core --hw sim
```


**Third Terminal**
We then offer the xbot2-gui to have a GUI to control and run plugins in the robot:

```
xbot2-gui
```

#### Docker

You can also run all the above using docker, following this: https://github.com/ADVRHumanoids/xbot2_examples?tab=readme-ov-file#running-inside-docker-container--

