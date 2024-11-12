# Doosan XBot2

This package contains the config files and instructions for running the Doosan robot with the XBot2 framework. It also contains a xbot2 example plugin to run a periodic motion on the robot.

At the moment, the package works with Ubuntu 20.04 and ROS Noetic.

# Dependencies

Install the following system dependencies:

- *ROS Noetic*
- *xbot2*: https://advrhumanoids.github.io/xbot2/master/index.html

Install the source dependencies (depending on the build tool you prefer):

- doosan-robot: https://github.com/doosan-robotics/doosan-robot

Let's first install its system dependencies (https://github.com/doosan-robotics/doosan-robot?tab=readme-ov-file#dependency-packages) and the source ones (e.g., serial), and then, if, for example, you are using catkin, it would be good to use the same workspace described below.

# How to run it?

## Setup and Configuration

Clone, compile, and install the repo (and its dependencies), for example, using a catkin package:

```
# create and initialize the catkin ws
mkdir -p magician_ws && cd magician_ws
mkdir -p src && cd src && catkin_init_workspace
 
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

## Run the Kinematic Simulation

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

[magician_xbot2.webm](https://github.com/user-attachments/assets/806277d1-bfb6-460d-9595-2fc77bd77b0a)


## Run the Dynamic Simulation on Gazebo

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

[magician_gazebo_xbot2.webm](https://github.com/user-attachments/assets/db8a54b0-9027-4aed-a3ab-4691d85b302b)

## Run the Position Cartesian Controller

We use CartesI/O https://advrhumanoids.github.io/CartesianInterface/ to obtain online Cartesian control of the Doosan Robot.

After you run the robot (either in simulation or on the real robot), you need to start the *ros_control* plugin to have access to the robot through ROS:

```
rosservice call /xbotcore/ros_control/switch 1
```

And then launch CartesI/O for the Doosan cartesian control:

```
roslaunch doosan_cartesio doosan.launch
```

[magician_cartesio.webm](https://github.com/user-attachments/assets/a785859e-17db-46f8-b98d-989910121a38)


# Docker

You can also run all the above using docker, following this: https://github.com/ADVRHumanoids/xbot2_examples?tab=readme-ov-file#running-inside-docker-container--

