cmake_minimum_required(VERSION 2.8.3)
project(iis_panda_controls)

find_package(catkin REQUIRED COMPONENTS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
    std_msgs
    message_generation
)

add_message_files(
  FILES
  BoundingBox.msg
  BB_Scene.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_install_python(PROGRAMS scripts/model_states_read.py scripts/joint_command_publisher.py scripts/read_cartesian_pose.py scripts/read_camera.py scripts/move_to_neutral.py scripts/benchmark_perception.py scripts/spawn_specific_scene.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



catkin_package(
  CATKIN_DEPENDS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)
