# POLIBRI Localization node
This is the localization node for the POLIBRI team.

## What is this node for?
This node is intended to estimate the localization of the drone using an EKF filter. The control variables are the linear and angular velocities of the drone w.r.t the map, and the state is composed by the drone's position w.r.t the map, and the markers' position w.r.t the map. 

The transform tree can be seen [here](resources/frames.pdf). All the odometry messages use the ENU or right hand convention, and all the measurement units are in meters or meters/secs for velocities

There are currently 2 nodes: one, **ekf_localization_node**, is responsible of the state estimation; and the other, **qr_code_node**, will stamp the messages related to the QR markers.
The qr_code_node node subscribes to the following messages:
* `/visp_auto_tracker/code_message` of type *std_msgs/String*
* `/visp_auto_tracker/stamped_message` of type *[ekf_localization/StringStamped](msg/StringStamped.msg)*
* `/visp_auto_tracker/object_position` of type *geometry_msgs/PoseStamped*

and publishes:
* `/visp_auto_tracker/stamped_object_position` of type [ekf_localization/QRCodeStamped](msg/QRCodeStamped.msg)
* `/visp_auto_tracker/stamped_message` of type [ekf_localization/StringStamped](msg/StringStamped.msg)

The ekf_localization_node subscribes to the following messages (*all these messages can be configured in [params.yaml](config/params.yaml)*):
* `/gazebo/ground_truth` (*nav_msgs/Odometry*) or `/mavros/local_position/odom` (*nav_msgs/Odometry*)
* `/visp_auto_tracker/stamped_object_position` ([ekf_localization/QRCodeStamped](msg/QRCodeStamped.msg))
* `/pole_localization` ([ekf_localization/RangeAndBearingPole.msg](msg/RangeAndBearingPole.msg))

and publishes
* `/drone/odom` (*nav_msgs/Odometry*)
* `/markers/pose` ([ekf_localization/QRCodeStampedArray](msg/QRCodeStampedArray.msg))

it also provides 3 services:
* `/get_drone_state` which provides the drone pose ([GetDroneState.srv](srv/GetDroneState.srv))
* `/get_marker_state` which provides the estimated pose of a given marker ([GetMarkerState.srv](srv/GetMarkerState.srv))
* `/save_map` which saves the landmarks map ([SaveMap.srv](srv/SaveMap.srv))

## Simulator setup
To install and run the simulator refer to the README in the Simulator folder of this repo. 

## Build and run the nodes
### Dependencies
There are some dependencies needed to compile the node.
* [Eigen 3](http://eigen.tuxfamily.org/)
* [Yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [Boost](https://www.boost.org/)

### Steps
Once you have set up the simulator, you can build the localization node. 
1. Clone the repo on your `~/catkin_ws/src` folder
1. If you are using catkin tools, from `~/catkin_ws` run `catkin build ekf_localization`. If you are using plain catkin, run `catkin_make --only-pkg-with-deps ekf_localization`.
1. In different terminals run:
   1. `roscd px4` and then, `no_sim=1 make px4_sitl_default gazebo`
   1. from `~/catkin_ws`, run `./launch_gazebo.sh`
   1. `roslaunch rtabmap_ros my_stereo_mapping_2.launch` (if you want to run rviz, you can add at the end `rviz:=true`)
   1. `roslaunch ekf_localization my_launch.launch`, this will run the node responsible of the state estimation and the one responsible of stamp the marker's information
   1. If you want to run the planner scripts, you can run in different terminals:
        1. `python Simulator/offboard/start_offboard.py`
        1. `python Simulator/offboard/path_generation.py`
        
        
### Build and run the nodes using a rosbag
The nodes can be debugged by launching the [debug.launch](launch/debug.launch) file. To do so, you just need to run the following: `$ roslaunch ekf_localization debug.launch`.
Several arguments can be passed to the launch file, like the folder to find a rosbag, the name of the rosbag, record a new rosbag, specify topics to record, launch rviz, etc.


