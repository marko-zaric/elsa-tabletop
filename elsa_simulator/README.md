# ELSA Simulator 

This package is an extention of the panda_simulator subpackage of [*Panda Simulator*][pandasim-repo] in order to incorperate the Intel RealSense D435 in to the Gazebo simulation. 

## Models

This folder holds a collection of models used to populate the tabletop scenario via world file. The benchmark objects and some additional models which are nice for manual interaction testing. 

<div class="panel panel-warning">
**Warning**
{: .panel-heading}
<div class="panel-body">
Make sure that Gazebo is able to access the models in this folder. Typically the model path is automatically set in the respective launch files but if Gazebo fails to load them check the export manually.
</div>
</div>

## Robot

In this folder the robot is described by a .xacro which basically combines Franka Emika Panda from `franka_panda_description` with the Intel RealSense D435 into one robot model. 

## Launch files

When working with the Gazebo simulation all launchfiles are located here. Each launch file is used for a different use case:

|launch file| use case|
|-----|-----|
|benchmark| launches Gazebo ready for a benchmarking test of the perception node|
|sim|launches a basic table top scenario without planner services in order to test perception manually |
|gazebo_perception|Called by every other launch file and is to start the camera sensor, object database and downsample the point cloud|
|gazebo_color_clustering_test|Perception color clustering test world with spawned objects all touching. Used to check validity of second dbscan in the color space. |
|sim_with_kinematics| Starts in addition to all the perceptive functionalities a moveIt sim group for predictive planning ad forward kinematics estimation. |



[pandasim-repo]: <https://github.com/justagist/panda_simulator>