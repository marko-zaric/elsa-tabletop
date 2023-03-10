# ELSA Panda Simulator [![ROS Version](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg?logo=ros)](https://ros.org/) [![Python 3.8.10+](https://img.shields.io/badge/python-3.8.10+-blue.svg?logo=python)](https://www.python.org/downloads/release/python-3810/) [![franka_ros_version](https://img.shields.io/badge/franka_ros-v0.7.1-blue.svg)](https://github.com/frankaemika/franka_ros) [![franka_ros_interface_version](https://img.shields.io/badge/franka_ros_interface-v0.7.1-yellow.svg)](https://github.com/justagist/franka_ros_interface)

This packege contains a Gazebo simulation environment and real-world control and perception equivalent for **ELSA** - Effective Learning of Social Affordances for Human-Robot Interaction (ANR/FWF AAPG, 2022-2026) **WP3: Social Affordances for Action Execution**. It is a recreation of the tabletop scenario of the paper [*Learning Social Affordances and Using Them for Planning by Uyanik et. al (2013)*][uyanik-paper] using a Franka Emika Panda robot with a topdown Intel RealSense D435 depth camera for perception.

## Hardware
The real-world tabletop scenario uses uses the following hardware:
- [Franka Emika Panda][panda-hardware]
- [Intel RealSense D435][intelrs-hardware]


## Simulator
The Gazebo simulator is based on the GitHub package [*Panda Simulator*][pandasim-repo] which provides exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*Franka ROS Interface*][fri-repo] package. A perception module using the [*Intel RealSense Gazebo ROS plugin*][gazebo_rs-repo] was added to the simulator, which observes the tabletop scene and publishes the state to a scene_description topic allowing a contiuous state-action interaction loop between the scene and the robot.

## Features

- Automated physical scene observation with detailed object information (bounding box, surface features).

- Services for inverse kinematics calculation using MoveIt! and joint control commands (Sim Only).

- Object database for effect clustering in scene changes

*See [change log](https://git.uibk.ac.at/c7031403/panda_simulator/blob/master/changeLog.md) for details about new feature updates.*

### Installation

#### Dependencies

- `pip install -r requirements.txt #(to install numpy and numpy-quaternion)` (or `pip3 install -r requirements.txt`)
- *libfranka* (`apt install ros-${ROS_DISTRO}-libfranka` or [install from source][libfranka-doc]).
- Most of the other basic dependencies can be met by running the following `apt-get` command: `apt install ros-$ROS_DISTRO-gazebo-ros-control ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools`.

The following dependencies can be installed using the `.rosinstall` file (instructions in next section: [Building the Package](#building-the-package)).

- [*franka-ros*][libfranka-doc]
- [*panda_moveit_config*](https://github.com/ros-planning/panda_moveit_config)
- [*Franka ROS Interface*][fri-repo] (branch [`v0.7.1-dev`](https://github.com/justagist/franka_ros_interface/tree/v0.7.1-dev) branch)
- [*franka_panda_description*][fpd-repo] (urdf and model files from *panda_description* package modified to work in Gazebo, with the custom controllers, and more realistic dynamics parameters)
- [*orocos-kinematics-dynamics*](https://github.com/orocos/orocos_kinematics_dynamics) (requires a specific commit; see instructions below)

**NOTE**: The franka_panda_description package above has to be independently updated regularly (using `git pull`) to get the latest robot description, visual and dynamics parameters.

#### Building the Package

1.Clone the repo:

```bash
    cd <catkin_ws>/src
    git clone -b noetic-devel https://github.com/justagist/panda_simulator
```

Steps 2 and 3 can be automated by running `./build_ws.sh` from `<catkin_ws>/src/panda_simulator`.

2.Update dependency packages:

```bash
    wstool init
    wstool merge panda_simulator/dependencies.rosinstall
    wstool up

    # use old ros-compatible version of kdl
    cd orocos_kinematics_dynamics && rm -rf * && git checkout b35c424e && git reset --hard
    cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys python-sip
```

3.Once the dependencies are met, the package can be installed using catkin_make:

```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)
    source devel/setup.bash
```

### Docker Build (experimental!)

**Requires [nvidia-docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)). Gazebo and RViz may not work without nvidia-docker.**

- To build the docker image of the package, run `docker build docker/ -t ps_${ROS_DISTRO}:v1.0.0`, or pull built image from github (`docker pull docker.pkg.github.com/justagist/panda_simulator/ps_${ROS_DISTRO}:v1.0.0`).
*Note: Even when using the docker image, this repository has to be cloned on to the host machine.*
- To run the built image interactively, run the script `./run_docker.sh` from the cloned repository. The container starts in a catkin workspace (directory location in host machine: `$HOME/.panda_sim_${ROS_DISTRO}_ws`). The host's home directory is also mounted in the container for access to `.ros/` and for making the catkin workspace writable. To see and modify other mounted volumes, go through the `run_docker.sh` file.
- When running for the first time, the catkin workspace has to be built (`cd src/panda_simulator && ./build_ws.sh`).
- If everything was successfully built in the previous step, you should be able to run the simulator (see [Usage](#usage) section below).

Any edits made to the host directory will be reflected in the docker container (and vice-versa). You can also run and build other ROS nodes and packages without having any ROS installation on the host machine.

### Usage

The simulator can be started by running:

```bash
    roslaunch panda_gazebo panda_world.launch # (use argument load_gripper:=false for starting without gripper; see other available arguments in launch file)
```

This exposes a variety of ROS topics and services for communicating with and controlling the robot in simulation. The robot can also be controlled using the [Franka ROS Interface](https://github.com/justagist/franka_ros_interface) package and/or [PandaRobot][pr-repo] APIs.

**Update: The above roslaunch command does not start the moveit server automatically anymore. If using Panda Simulator in ROS Melodic environment, the moveit server now has to be started manually by running the following command in a new terminal:**

```bash
    roslaunch panda_sim_moveit sim_move_group.launch # (use argument load_gripper:=false for starting without gripper
```

For known errors and issues, please see [Issues](#known-issues) section below.

#### Demos

To run these demos, launch the simulator first: `roslaunch panda_gazebo panda_world.launch`. The following demos can then be tested:

- Moveit Demo: The moveit server must be running (see [usage](#usage)). Run `roslaunch panda_simulator_examples demo_moveit.launch` to run a demo for testing the moveit planner interface with the simulated robot. This script starts a moveit RViz GUI for motion planning and terminal interface for modifying planning scene.

- Task-space control using Franka ROS Interface (or PandaRobot) API: Run `roslaunch panda_simulator_examples demo_task_space_control.launch` to run a demo showing the task-space control. By default, the demo uses the (Franka ROS Interface) API to retrieve state information, and to control it using torque control (see [script](panda_simulator_examples/scripts/task_space_control_with_fri.py)).

- Task-space control using ROS topics directly: Another script demonstrating the same functionality without using the Franka ROS Interface API, and only the ROS topics from the simulation is also [provided](panda_simulator_examples/scripts/task_space_control_using_sim_only.py). This can be run interactively by running `roslaunch panda_simulator_examples demo_task_space_control.launch use_fri:=False`.

- API usage demo: Another (much simpler) demo ['move_robot.py'](panda_simulator_examples/scripts/task_space_control_using_sim_only.py) is provided demonstrating (i) controlling the robot in the joint space, (ii) retrieving state information of the robot.

##### Task-space Impedance Control Demo

  ![vid](assets/ts_demo.gif)
 Watch video [here](https://youtu.be/a_HEmYzqEnk)

#### Some useful ROS topics

##### Published Topics

| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/custom_franka_state_controller/robot_state* | gravity, coriolis, jacobian, cartesian velocity, etc. |
| */panda_simulator/custom_franka_state_controller/tip_state* | end-effector pose, wrench, etc. |
| */panda_simulator/joint_states* | joint positions, velocities, efforts |

##### Subscribed Topics

| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/motion_controller/arm/joint_commands* | command the robot using the currently active controller |
| */panda_simulator/franka_gripper/move* | (action msg) command the joints of the gripper |

Other topics for changing the controller gains (also dynamically configurable), command timeout, etc. are also available.

#### ROS Services

Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient (via the same topics as the real robot and [*franka_ros*][franka-ros]).

### Known Issues

1. `[ERROR] Exception while loading planning adapter plugin 'default_planner_request_adapters/ResolveConstraintFrames` in melodic. This can be [safely ignored](https://github.com/ros-planning/moveit/issues/1655).

2. `Error in REST request` error message when starting Gazebo. This also can be safely ignored, or fixed by following the instructions [here](https://answers.gazebosim.org/question/25030/gazebo-error-restcc205-error-in-rest-request/?answer=25048#post-id-25048).

3. ~Gripper control and model definition is not completely developed, and gripper control may not produce the required performance.~ *Update: robot and gripper model definitions have now been improved in the [franka_panda_description][fpd-repo] package*.

4. Gravity compensation when using velocity or torque controller with gripper is not very good. This is bypassed by deactivating simulator gravity by default (see [`panda.world`](panda_gazebo/worlds/panda.world)).

### Version Update

Check [versionLog.md](https://github.com/justagist/panda_simulator/blob/noetic-devel/versionLog.md).

## Related Packages

- [*Franka ROS Interface*][fri-repo] : A ROS API for controlling and managing the Franka Emika Panda robot (real and simulated). Contains controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [*PandaRobot*][pr-repo] : Python interface providing higher-level control of the robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).
- [*Gazebo Panda*](https://github.com/justagist/gazebo_panda): A simple bare-bone gazebo simulator using in-built gazebo controllers and transmissions. No custom controllers or interfaces.

The [*Franka ROS Interface*][fri-repo] package provides Python API and interface tools to control and communicate with the robot using the ROS topics and services exposed by the simulator. Since the simulator exposes similar information and controllers as the *robot_state_controller_node* of the [*Franka ROS Interface*][fri-repo], the API can be used to control both the real robot, and the simulated robot in this package, with minimum change in code.

### License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2021, Saif Sidhik

If you use this software, please cite it using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747459.svg)](https://doi.org/10.5281/zenodo.3747459).

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
