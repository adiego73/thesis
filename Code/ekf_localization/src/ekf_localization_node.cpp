#include "ekf_localization/ekf_slam.hpp"
#include <ros/ros.h>

int
main(int argc, char **argv)
{

    ros::init(argc, argv, "ekf_localization");

    ROS_INFO(":: EKF LOCALIZATION node instantiated ::");
    std::map<std::string, std::string> parameters;

    if (!ros::param::get("/ekf_localization_node/odometry_topic", parameters["odometry_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/odometry_topic");
    }

    if (!ros::param::get("/ekf_localization_node/pole_landmark_topic", parameters["pole_landmark_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/pole_landmark_topic");
    }

    if (!ros::param::get("/ekf_localization_node/drone_odom_topic", parameters["drone_odom_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/drone_odom_topic");
    }

    if (!ros::param::get("/ekf_localization_node/markers_pose_topic", parameters["markers_pose_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/markers_pose_topic");
    }

    if (!ros::param::get("/ekf_localization_node/marker_landmark_topic", parameters["marker_landmark_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/marker_landmark_topic");
    }

    if (!ros::param::get("/ekf_localization_node/map_path", parameters["map_path"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/map_path");
    }

    if (!ros::param::get("/ekf_localization_node/octomap_path", parameters["octomap_path"]))
    {
        ROS_WARN("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/octomap_path");
    }

    if (!ros::param::get("/ekf_localization_node/octomap_topic", parameters["octomap_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/octomap_topic");
    }

    if (!ros::param::get("/ekf_localization_node/range_sensor_topic", parameters["range_sensor_topic"]))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/ekf_localization/range_sensor_topic");
    }

    bool enable_poles;
    ros::param::get("/ekf_localization_node/enable_poles_subscriber", enable_poles);
    parameters["enable_poles"] = enable_poles ? "true" : "false";

    bool enable_markers;
    ros::param::get("/ekf_localization_node/enable_markers_subscriber", enable_markers);
    parameters["enable_markers"] = enable_markers ? "true" : "false";

    bool enable_laser;
    ros::param::get("/ekf_localization_node/enable_range_subscriber", enable_laser);
    parameters["enable_laser"] = enable_laser ? "true" : "false";

    // the state vector is composed by: [ x y z yaw | (for each marker: x y z) ]
    EKFSLAM localization(4, parameters);

    // run at 30Hz.
    ros::Rate rate(30);
    while (ros::ok())
    {

        localization.run();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}