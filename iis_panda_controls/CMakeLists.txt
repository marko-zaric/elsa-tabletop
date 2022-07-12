cmake_minimum_required(VERSION 2.8.3)
project(iis_panda_controls)

find_package(catkin REQUIRED COMPONENTS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
    iis_panda_controls
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)

catkin_python_setup()
catkin_install_python(PROGRAMS scripts/model_states_read.py scripts/joint_command_publisher.py scripts/read_cartesian_pose.py scripts/read_camera.py scripts/move_to_neutral.py scripts/benchmark_perception.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
