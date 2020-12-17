#include <cmath>
#include <random>
#include "ekf_localization/commons.hpp"

std::mt19937 generator(std::random_device{}());
std::normal_distribution<double> dist(0, 1e-2);
std::normal_distribution<double> azim(0, 1e-5);
std::normal_distribution<double> elev(0, 1e-3);

//int next_pole = 0;

struct Pole
{
    int id;
    double x;
    double y;
    double z;
};

const Pole poles[6] = {{1, 9.7, -4.7, 3}
                       , {2, 9.7, 4.7, 3}
                       , {3, 0, 4.7, 3}
                       , {4, 0, -4.7, 3}
                       , {5, -9.7, -4.7, 3}
                       , {6, -9.7, 4.7, 3}};

void
robotPoseCallback(const nav_msgs::OdometryConstPtr &odometry_msg, const ros::Publisher &pub, const ros::Publisher &p_visualization);

int
main(int argc, char **argv)
{
    std::srand(std::time(nullptr));
    ros::init(argc, argv, "fake_pole_localization");

    ROS_INFO(":: FAKE POLE LOCALIZATION node instantiated ::");

    ros::NodeHandle node;

    std::string odometry_topic, pole_localization_topic;

    if (!ros::param::get("/fake_pole_localization/odometry_topic", odometry_topic))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/fake_pole_localization/odometry_topic");
    }

    if (!ros::param::get("/fake_pole_localization/pole_localization_topic", pole_localization_topic))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/fake_pole_localization/pole_localization_topic");
    }

    ros::Publisher p_visualization = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher p_pole_observation = node.advertise<poles_vision::RangeAndBearingPole>(pole_localization_topic, 1);
    ros::Subscriber s_drone_pose = node.subscribe<nav_msgs::Odometry>(odometry_topic, 1, boost::bind(&robotPoseCallback, _1, std::ref(p_pole_observation), std::ref(p_visualization)));

    ros::Rate frequency(20);
    while (ros::ok())
    {
        ros::spinOnce();
        frequency.sleep();
    }
}

void
robotPoseCallback(const nav_msgs::OdometryConstPtr &odometry_msg, const ros::Publisher &pub, const ros::Publisher &debug)
{
    int next_pole = (std::rand() % 6);
//
    Pole p = poles[next_pole];
    // send poles in order
//    next_pole = (next_pole + 1 > 5 ? 0 : next_pole + 1);

    auto robot_position = odometry_msg->pose.pose.position;
    auto robot_orientation = odometry_msg->pose.pose.orientation;

    double roll, pitch, yaw;
    tf2::Quaternion q_orientation(robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w);
    tf2::Matrix3x3(q_orientation).getRPY(roll, pitch, yaw);

    // assumes that the robot is in the odom frame, even when the information comes from the ground truth.
    // This is because the information of the range and bearing comes from the real drone's position.
    Eigen::Matrix4d R;
    get_homogeneous_transformation(robot_position.x, robot_position.y, robot_position.z, roll, pitch, yaw, R);

    /*for(auto p : poles)*/ {
        poles_vision::RangeAndBearingPole message;
        message.header.stamp = ros::Time::now();
        message.id = p.id;

        Eigen::Vector4d pole_pos;
        pole_pos(0) = p.x;
        pole_pos(1) = p.y;
        pole_pos(2) = p.z;
        pole_pos(3) = 1;

        // transforms the poles position from the map reference frame to the drone's reference frame.
        Eigen::Vector4d vec_hat = R.inverse() * pole_pos;

        // the distance is calculated as a straight line from the drone to the pole, hence in the same plane as the drone (I do not consider z).
        message.distance = std::sqrt(std::pow(vec_hat(0), 2) + std::pow(vec_hat(1), 2)) + dist(generator);
        message.azimuth = std::atan2((vec_hat(1)), (vec_hat(0))) + azim(generator);
        message.elevation = std::atan2(vec_hat(2), message.distance) + elev(generator);

        message.trust[0] = message.trust[1] = message.trust[2] = 1;

        pub.publish(message);
    }
}