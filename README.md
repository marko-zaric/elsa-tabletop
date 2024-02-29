# ELSA Panda Simulator [![ROS Version](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg?logo=ros)](https://ros.org/) [![Python 3.8.10+](https://img.shields.io/badge/python-3.8.10+-blue.svg?logo=python)](https://www.python.org/downloads/release/python-3810/) [![franka_ros_version](https://img.shields.io/badge/franka_ros-v0.7.1-blue.svg)](https://github.com/frankaemika/franka_ros) [![franka_ros_interface_version](https://img.shields.io/badge/franka_ros_interface-v0.7.1-yellow.svg)](https://github.com/justagist/franka_ros_interface)

This package contains a Gazebo simulation environment and real-world control and perception equivalent for **ELSA** - Effective Learning of Social Affordances for Human-Robot Interaction (ANR/FWF AAPG, 2022-2026) **WP3: Social Affordances for Action Execution**. It is a recreation of the tabletop scenario of the paper [*Learning Social Affordances and Using Them for Planning by Uyanik et. al (2013)*][uyanik-paper] using a Franka Emika Panda robot with a topdown Intel RealSense D435 depth camera for perception.

## Hardware
The real-world tabletop scenario uses the following hardware:
- [Franka Emika Panda][panda-hardware]
- [Intel RealSense D435][intelrs-hardware]


## Simulator
The Gazebo simulator is based on the GitHub package [*Panda Simulator*][pandasim-repo] which provides exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*Franka ROS Interface*][fri-repo] package. A **perception module** using the [*Intel RealSense Gazebo ROS plugin*][gazebo_rs-repo] was added to the simulator, which observes the tabletop scene and publishes the state to a scene_description topic allowing a contiuous state-action interaction loop between the scene and the robot.

## Features Overview

- Automated physical scene observation with detailed object information (pose, bounding box, surface features)

- Services for inverse kinematics calculation using MoveIt! and joint control commands (Sim Only)

- Object database for effect clustering in scene changes

*See [change log](https://git.uibk.ac.at/c7031403/panda_simulator/blob/master/changeLog.md) for details about new feature updates.*

## Installation

### Dependencies

#### Python

- `pip install -r requirements.txt'` (or `pip3 install -r requirements.txt`)

#### ROS packages

- *libfranka* (`apt install ros-noetic-libfranka` or [install from source][libfranka-doc]).
- Most of the other basic dependencies can be met by running the following `apt-get` command: `apt install ros-noetic-franka-ros ros-noetic-gazebo-ros-control ros-noetic-rospy-message-converter ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools ros-noetic-ddynamic-reconfigure`.

#### Intel RealSense

```bash
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
    apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg ros-noetic-realsense2-description 
```

### Building the Package

1.Clone the repo:

```bash
    cd <catkin_ws>/src
    git clone https://git.uibk.ac.at/c7031403/panda_simulator.git
    git submodule update --init --recursive
```

2.Build repository:

```bash
    source /opt/ros/noetic/setup.bash
    catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)
    source devel/setup.bash
```
## Overview
This package contains several subpackages for convenient use. All packages starting with *elsa_<name\>* are packages specifially developed for this research project and aim to solve separate problems or tasks.

| Package | Description |
| ------ | ------ |
| elsa_benchmark | Benchmark package specifically designed for benchmarking *elsa_perception* |
| elsa_object_database | A collection of registered colored objects to help track feature changes |
| elsa_panda_controls | A collection of nodes and serices with the purpose of easy control in joint and cartesian space |
| elsa_perception | Package containing nodes for obtaining point cloud information of the real IntelRealSense camera or the Gazebo simulated one and subsequent object segmentation  |
| elsa_perception_msgs | ROS messages used by the *elsa_perception* package |
| elsa_simulator | Package dedicated to the simulation of the setup in Gazebo including launch and world files as well as object models. |
| elsa_training_data_generation | Data generation package for SVM training currently only for object detection based on feature vectors described in the Uyanik paper |

Inherited Packages from GitHub package [*Panda Simulator*][pandasim-repo]:

| Package | Description |
| ------ | ------ |
| franka_panda_description | Robot description package modified from franka_ros package to include dynamics parameters for the robot arm and gripper for simulating the behaviour of the real robot. |
| panda_gazebo | Package dedicated to the simulation of the Franka Emika Panda in Gazebo including launch and world files as well as object models. |
| panda_hardware_interface | See GitHub package [*Panda Simulator*][pandasim-repo] |
| panda_sim_controllers | See GitHub package [*Panda Simulator*][pandasim-repo] |
| panda_sim_custom_action_server |  |
| panda_sim_moveit | See GitHub package [*Panda Simulator*][pandasim-repo] |
| panda_simulator | See GitHub package [*Panda Simulator*][pandasim-repo] |
| panda_simulator_examples | See GitHub package [*Panda Simulator*][pandasim-repo] |

Submodules:

| Package | Description |
| ------ | ------ |
| franka_ros | The Franka Control Interface (FCI) allows a fast and direct low-level bidirectional connection to the Arm and Hand. It provides the current status of the robot and enables its direct control with an external workstation PC connected via Ethernet. |
| franka_ros_interface | A ROS interface library for the Franka Emika Panda robot (real and simulated), extending the franka-ros library to expose more information about the robot, and providing low-level control of the robot using ROS and Python API. |
| orocos_kinematics_dynamics | Orocos project to supply RealTime usable kinematics and dynamics code, it contains code for rigid body kinematics calculations and representations for kinematic structures and their inverse and forward kinematic solvers. |
| panda_moveit_config | The Panda robot is the flagship MoveIt integration robot used in the MoveIt tutorials. Any changes to MoveIt need to be propagated into this config fast, so this package is co-located under the ros-planning |
| realsense-ros |  Intel RealSense ROS1 Wrapper |
| realsense_gazebo_plugin | This package is a Gazebo ROS plugin for the Intel D435 realsense camera. |


## Usage
To use this package with the Gazebo simulation or the real-world scenario different launch procedures are in place.
### Gazebo Simulator

#### **1. Perception only**

The simulator with only perception (no inverse kinematics) can be started by running:

```bash
    roslaunch elsa_simulator sim.launch # (use argument load_gripper:=false for starting without gripper; see other available arguments in launch file)
```

This exposes a variety of ROS topics and services with data regarding the observed tabletop scene.

***Published Topics***

| ROS Topic | Data |
| ------ | ------ |
| */scene_description<_stamped>* | percived objects in scene with respective information (pose, bounding box, surface features, identiy) |
| */fullscene<_stamped>* | scene_description topic with dummy social features |


#### **2. Full simulation with IK and joint controller**
This launch file starts the full simulation including the previously descibed perception functionalities as well as services for inverse kinematics calculation using MoveIt! and a joint controller.

```bash
    roslaunch elsa_simulator sim_with_kinematics.launch # (use argument load_gripper:=false for starting without gripper; see other available arguments in launch file)
```

***ROS Services***

| ROS Service | Usage |
| ------ | ------ |
| *calc_ik* | calculates goal joint angles from given goal cartesian position in 3D (perserves orientation) or 6D (xyz + orientation) |
| *send_joint_config* | moves robot to specific joint configuration |

Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient (via the same topics as the real robot and [*franka_ros*][franka-ros]).

### License
This project is licensed under a Creative Commons Attribution 4.0 International License - see the [LICENSE](LICENSE) file for details.

### Acknowledgements
This research was funded in whole or in part by the Austrian Science Fund (FWF) [ELSA, DOI: 10.55776/I5755]. For open access purposes, the author has applied a CC BY public copyright license to any author accepted manuscript version arising from this submission.

   [fri-repo]: <https://github.com/justagist/franka_ros_interface>
   [pd_simulator-repo]: <https://github.com/justagist/franka_panda_description>
   [pandasim-repo]: <https://github.com/justagist/panda_simulator>
   [uyanik-paper]: <https://escholarship.org/content/qt9cj412wg/qt9cj412wg.pdf>
   [pr-repo]: <https://github.com/justagist/panda_robot>
   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://frankaemika.github.io/docs/franka_ros.html>
   [intelrs-hardware]: <https://www.intelrealsense.com/depth-camera-d435/>
   [panda-hardware]: <https://www.franka.de/>
   [gazebo_rs-repo]: <https://github.com/pal-robotics/realsense_gazebo_plugin>
