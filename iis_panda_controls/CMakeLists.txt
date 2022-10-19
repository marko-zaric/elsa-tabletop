cmake_minimum_required(VERSION 2.8.3)
project(iis_panda_controls)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
    std_msgs
    message_generation
    genmsg
)

find_package (OpenCV 4.0.0 REQUIRED)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  BoundingBox.msg
  BB_Scene.msg
  Feature.msg
  FeatureVector.msg
)

add_service_files(
  FILES
  SurfaceFeatures.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
  CATKIN_DEPENDS
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)

catkin_install_python(PROGRAMS scripts/node_gazebo_model_states_read.py 
                      scripts/node_gazebo_move_to_neutral.py 
                      scripts/node_gazebo_read_camera.py 
                      scripts/node_gazebo_read_cartesian_pose.py 
                      scripts/node_gazebo_joint_command_publisher.py 
                      scripts/node_gazebo_read_camera.py 
                      scripts/test_point_cloud.py
                      scripts/test_bounding_boxes.py
                      scripts/test_surface_normals.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## PC C++ Nodes
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(subscribe_point_cloud src/sub_pc.cpp)
target_link_libraries(subscribe_point_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(save_point_cloud src/save_point_cloud.cpp)
target_link_libraries(save_point_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(surface_feature_service src/surface_features_service.cpp)
target_link_libraries(surface_feature_service ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

