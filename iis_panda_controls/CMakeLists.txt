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

catkin_install_python(PROGRAMS scripts/node_gazebo_model_states_read.py 
                      scripts/node_gazebo_move_to_neutral.py 
                      scripts/node_gazebo_read_camera.py 
                      scripts/node_gazebo_read_cartesian_pose.py 
                      scripts/node_gazebo_joint_command_publisher.py 
                      scripts/node_gazebo_read_camera.py 
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
