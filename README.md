# Doosan XBot2

This package contains the config files and the instructions to run the Doosan robot with the XBot2 framework.

At the moment the package works with Ubuntu 20.04 and ROS Noetic.

## Dependencies

Install the following system dependencies:

- ROS Noetic
- xbot2: https://advrhumanoids.github.io/xbot2/master/index.html

Include in the sources dependencies (depending on the build tool you prefer, the following pacakge):

- doosan-robot: https://github.com/doosan-robotics/doosan-robot

Let's first install its system dependencies (https://github.com/doosan-robotics/doosan-robot?tab=readme-ov-file#dependency-packages) and then, if for example you are using catkin it would be good to use the same workspace described below.

## How to run it?

### Setup and Configuration

Clone, compile and install the repo (and its dependencis) for example using a catkin package:


```
# create and initialize the catkin ws
mkdir -p magician_ws && cd magician_ws
mkdir -p src && catkin_init_workspace
 
# assuming you are using https
git clone https://github.com/magician-project/doosan_xbot2.git

# compile it
cd .. && catkin_make

```
