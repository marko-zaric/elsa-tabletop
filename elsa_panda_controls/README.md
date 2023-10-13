# ELSA Panda Controls 

This package offers a variety of nodes and services for easy control of the Franka Emika Panda robot in simulation. It enables task space and joint space control as well as some predefined gestures for testing human robot social interactions.

## Services

| ROS Service | Usage |
| ------ | ------ |
| *calc_ik* | Calculates goal joint angles from given goal cartesian position in 3D (perserves orientation) or 6D (xyz + orientation) |
| *send_joint_config* | Moves robot to specific joint configuration |

Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient (via the same topics as the real robot and [*franka_ros*][franka-ros]).

## Standalone Nodes

### Task Space Commands

Task space commands are performed in cartesian space with kinematic control there are various nodes that accept arguments on startup which perform the specified movement.

| Ros Node | Description | Usage Example  |
| ------ | ------ | ------ |
| *node_gazebo_joint_command_publisher* | Takes in a goal cartesian position in 6D (xyz + orientation as quaternion) | `rosrun elsa_panda_controls node_gazebo_joint_command_publisher -p 0.3 0.3 0.3 -o 0.0 -1.0 0.0 0.0`
| *node_gazebo_move_catesian* | Same as *node_gazebo_joint_command_publisher* but takes in only 3D cartesian position while keeping the start positions orientation. | `rosrun elsa_panda_controls node_gazebo_move_catesian -p 0.3 0.3 0.3` |
| *node_gazebo_read_cartesian_pose* | Returns endeffectors current cartesian position and orientation as an quaternion. | `rosrun elsa_panda_controls node_gazebo_read_cartesian_pose`

### Joint Space Commands

These nodes use the joint space controller to move the robot into a specific configuration.

| Ros Node | Description | Usage Example  |
| ------ | ------ | ----- |
| *node_gazebo_move_to_neutral* |  Moves the robot into the neutral position out out of the view of the camera. | `rosrun elsa_panda_controls node_gazebo_move_to_neutral.py` 
| *node_gazebo_move_to_zero_config* | Moves the robot into the zero configuration which is straight up all joints extended to the sealing. | `rosrun elsa_panda_controls node_gazebo_move_to_zero_config.py`
| *node_gazebo_test_joint_movement* | Moves through the whole range of motion of one desired joint from joint limit to joint limit. | `rosrun elsa_panda_controls node_gazebo_test_joint_movement.py --joint panda_joint1` |
| *node_gazebo_single_joint_control* | Changes the joint value of one single joint in joint space. | `rosrun elsa_panda_controls node_gazebo_single_joint_control.py --joint panda_joint1 --value 0` |

### Gestures
These gesture primitives where hand designed in order to demostrate potential noverbal communication from robot to human.

| Ros Node | Description |
| ------ | ------ |
| *node_gazebo_pass_gesture* | Gestures to pass an object by flicking the wrist. |
| *node_gazebo_point_gesture* | Points to a object on the table. Takes in arguments of objects position. |
| *node_gazebo_grabby_motion* | Opens and closes the gripper to signalize the desire to grab an object. |