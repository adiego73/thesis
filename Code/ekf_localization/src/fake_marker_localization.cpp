#include <cmath>
#include <random>
#include "ekf_localization/commons.hpp"

std::mt19937 generator(std::random_device{}());
std::normal_distribution<double> noise(0, 0.01);

struct Marker
{
    int id;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};
const Marker markers[10] = {{0, -2, 1, 0.1, 0.0, 0.0, 0.0}
                            , {1, 3, 2, 0.1, 0.0, 0.0, 0.0}
                            , {2, 6, -3, 0.1, 0.0, 0.0, 0.0}
                            , {3, 6, 1, 0.1, 0.0, 0.0, 0.0}
                            , {4, -2, -2, 0.1, 0.0, 0.0, 0.0}
                            , {5, 1, 0, 0.1, 0.0, 0.0, 0.0}
                            , {6, -6, 0, 0.1, 0.0, 0.0, 0.0}
                            , {7, 0, 3, 0.1, 0.0, 0.0, 0.0}
                            , {8, 2, -3, 0.1, 0.0, 0.0, 0.0}
                            , {9, -6, -4, 0.1, 0.0, 0.0, 0.0}};

void
robotPoseCallback(const nav_msgs::OdometryConstPtr &odometry_msg, const int &landmark_id, nav_msgs::Odometry &odom, const ros::Publisher &p, const ros::Publisher &p2);

void
landmarkCallback(const std_msgs::StringConstPtr &data, int &landmark_id);

void
rvizDrawAllMarkers(const ros::Publisher &p_marker);

void
rvizDrawSeenMarker(const geometry_msgs::PoseStampedConstPtr &pose_msg, nav_msgs::Odometry &odom, const ros::Publisher &p_rviz);

int
main(int argc, char **argv)
{
    std::srand(std::time(nullptr));
    ros::init(argc, argv, "fake_marker_localization");

    ROS_INFO(":: FAKE MARKER LOCALIZATION node instantiated ::");

    ros::NodeHandle node;

    std::string odometry_topic, marker_localization_topic;

    if (!ros::param::get("/fake_marker_localization/odometry_topic", odometry_topic))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/fake_marker_localization/odometry_topic");
    }

    if (!ros::param::get("/fake_marker_localization/marker_localization_topic", marker_localization_topic))
    {
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), "/fake_marker_localization/marker_localization_topic");
    }

    int landmark_id = -1;
    nav_msgs::Odometry odom;

    ros::Publisher p_marker_observation = node.advertise<ekf_localization::QRCodeStamped>(marker_localization_topic, 1);
    ros::Publisher p_marker_pose = node.advertise<geometry_msgs::PoseStamped>("/fake/marker/pose", 1);
//    ros::Publisher p_real_marker_to_local = node.advertise<geometry_msgs::PoseStamped>("/estimated/real_marker/pose", 1);

    ros::Publisher p_real_viz_marker = node.advertise<visualization_msgs::Marker>("/visualization/qr_maker", 10);
    ros::Publisher p_seen_viz_marker = node.advertise<visualization_msgs::Marker>("/visualization/qr_maker_seen", 10);
    ros::Subscriber s_real_marker_pose = node.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position", 10, boost::bind(&rvizDrawSeenMarker, _1, std::ref(odom), std::ref(p_seen_viz_marker)));

    ros::Subscriber s_drone_pose = node.subscribe<nav_msgs::Odometry>(odometry_topic, 10, boost::bind(&robotPoseCallback, _1, std::ref(landmark_id), std::ref(odom), std::ref(p_marker_observation), std::ref(p_marker_pose)));
    ros::Subscriber s_marker_pose = node.subscribe<std_msgs::String>("/visp_auto_tracker/code_message", 10, boost::bind(&landmarkCallback, _1, std::ref(landmark_id)));

    ros::Rate r(30);
    while (ros::ok())
    {

        rvizDrawAllMarkers(p_real_viz_marker);

        ros::spinOnce();
        r.sleep();
    }
//    ros::spin();
}

void
landmarkCallback(const std_msgs::StringConstPtr &data, int &landmark_id)
{
    if (data->data.empty())
    {
        landmark_id = -1;
        return;
    }

    std::string code = data->data.substr(data->data.find_last_of(' '));
    try
    {
        landmark_id = std::stoi(code);
    }
    catch (std::exception &e)
    {
        landmark_id = -1;
        ROS_WARN("%s", e.what());
        return;
    }
}

void
robotPoseCallback(const nav_msgs::OdometryConstPtr &odometry_msg, const int &landmark_id, nav_msgs::Odometry &odom, const ros::Publisher &p, const ros::Publisher &p2)
{
    if (landmark_id < 0 || landmark_id > 9)
    {
        return;
    }

    odom.pose = odometry_msg->pose;

    Marker marker = markers[landmark_id];
    auto robot_orientation = odometry_msg->pose.pose.orientation;
    auto robot_position = odometry_msg->pose.pose.position;

    Eigen::Vector3d euler;
    tf2::Quaternion q_orientation(robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w);
    get_euler_from_quaternion(q_orientation, euler);

    // transform robot
    Eigen::Matrix4d Tr;
    get_homogeneous_transformation(robot_position.x, robot_position.y, robot_position.z, euler(0), euler(1), euler(2), Tr);

    Eigen::Matrix4d Tc;
    get_homogeneous_transformation(0, 0, 0, -M_PI, 0, -M_PI_2, Tc);

    Eigen::Matrix4d M;
    get_homogeneous_transformation(marker.x, marker.y, marker.z, marker.roll, marker.pitch, marker.yaw, M); // no rotation

    Eigen::Matrix4d T = Tr * Tc;
    // transforms the marker position from the map reference frame to the drone's reference frame.
    Eigen::Matrix4d M_local = T.inverse() * M;

    std::stringstream data;
    data << "qr marker with code " << marker.id;

    ekf_localization::QRCodeStamped message;

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";

    message.marker.code = marker.id;
    message.marker.data = data.str();
    message.marker.pose.position.x = M_local(0, 3) + noise(generator);
    message.marker.pose.position.y = M_local(1, 3) + noise(generator);
    message.marker.pose.position.z = M_local(2, 3) + noise(generator);

    tf2::Quaternion m_quaternion;
    get_quaternion_from_matrix(M_local.block<3, 3>(0, 0), m_quaternion);
    m_quaternion.setX(m_quaternion.getX() + noise(generator));
    m_quaternion.setY(m_quaternion.getY() + noise(generator));
    m_quaternion.setZ(m_quaternion.getZ() + noise(generator));
    m_quaternion.setW(m_quaternion.getW() + noise(generator));
    message.marker.pose.orientation = tf2::toMsg(m_quaternion);

    geometry_msgs::PoseStamped pose;
    pose.header = message.header;
    pose.pose = message.marker.pose;

    p2.publish(pose);
    p.publish(message);
}

void
rvizDrawAllMarkers(const ros::Publisher &p_marker)
{
    for (const auto &qr : markers)
    {
        visualization_msgs::Marker rviz_marker;
        rviz_marker.header.stamp = ros::Time::now();
        rviz_marker.header.frame_id = "map";

        rviz_marker.ns = "qr_marker";
        rviz_marker.id = qr.id;

        rviz_marker.type = visualization_msgs::Marker::CUBE;
        rviz_marker.action = visualization_msgs::Marker::ADD;

        rviz_marker.pose.position.x = qr.x;
        rviz_marker.pose.position.y = qr.y;
        rviz_marker.pose.position.z = qr.z;

        rviz_marker.pose.orientation.x = 0.0;
        rviz_marker.pose.orientation.y = 0.0;
        rviz_marker.pose.orientation.z = 0.0;
        rviz_marker.pose.orientation.w = 1.0;

        rviz_marker.scale.x = 0.2;
        rviz_marker.scale.y = 0.2;
        rviz_marker.scale.z = 0.01;

        rviz_marker.color.a = 0.7;
        rviz_marker.color.r = 0.0;
        rviz_marker.color.g = 1.0;
        rviz_marker.color.b = 0.0;

        rviz_marker.lifetime = ros::Duration();

        visualization_msgs::Marker text;
        text.header.stamp = ros::Time::now();
        text.header.frame_id = "map";

        text.ns = "qr_marker";
        text.id = qr.id + 10;

        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;

        text.pose.position.x = qr.x;
        text.pose.position.y = qr.y;
        text.pose.position.z = qr.z;

        text.pose.orientation.x = 0.0;
        text.pose.orientation.y = 0.0;
        text.pose.orientation.z = 0.0;
        text.pose.orientation.w = 1.0;

        text.scale.z = 0.2;

        text.color.a = 1.0;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;

        text.text = "Marker: " + std::to_string(qr.id);

        text.lifetime = ros::Duration();

        p_marker.publish(rviz_marker);
        p_marker.publish(text);
    }
}

void
rvizDrawSeenMarker(const geometry_msgs::PoseStampedConstPtr &pose_msg, nav_msgs::Odometry &odom, const ros::Publisher &p_rviz)
{
    Eigen::Vector3d euler;
    tf2::Quaternion q_drone;
    q_drone.setX(odom.pose.pose.orientation.x);
    q_drone.setY(odom.pose.pose.orientation.y);
    q_drone.setZ(odom.pose.pose.orientation.z);
    q_drone.setW(odom.pose.pose.orientation.w);
    get_euler_from_quaternion(q_drone, euler);

    Eigen::Matrix4d Tr;
    get_homogeneous_transformation(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, euler(0), euler(1), euler(2), Tr);

    Eigen::Matrix4d Tc;
    get_homogeneous_transformation(0, 0, 0, -M_PI, 0, -M_PI_2, Tc);

    Eigen::Matrix4d Tm;
    Eigen::Vector3d m_euler;
    tf2::Quaternion q_marker;
    q_marker.setX(pose_msg->pose.orientation.x);
    q_marker.setY(pose_msg->pose.orientation.y);
    q_marker.setZ(pose_msg->pose.orientation.z);
    q_marker.setW(pose_msg->pose.orientation.w);
    get_euler_from_quaternion(q_drone, m_euler);

    get_homogeneous_transformation(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z, m_euler(0), m_euler(1), m_euler(2), Tm);

    Eigen::Matrix4d Tw_m = Tr * Tc * Tm;

    tf2::Quaternion q_drone_world;
    get_quaternion_from_matrix(Tw_m.block<3, 3>(0, 0), q_drone_world);

    visualization_msgs::Marker rviz_marker;
    rviz_marker.header.stamp = ros::Time::now();
    rviz_marker.header.frame_id = "map";

    rviz_marker.ns = "qr_marker_seen";

    rviz_marker.type = visualization_msgs::Marker::CUBE;
    rviz_marker.action = visualization_msgs::Marker::ADD;

    rviz_marker.pose.position.x = Tw_m(0, 3);
    rviz_marker.pose.position.y = Tw_m(1, 3);
    rviz_marker.pose.position.z = Tw_m(2, 3);

    rviz_marker.pose.orientation = tf2::toMsg(q_drone_world);

    rviz_marker.scale.x = 0.2;
    rviz_marker.scale.y = 0.2;
    rviz_marker.scale.z = 0.01;

    rviz_marker.color.a = 0.7;
    rviz_marker.color.r = 0.0;
    rviz_marker.color.g = 0.0;
    rviz_marker.color.b = 1.0;

    rviz_marker.lifetime = ros::Duration();

    p_rviz.publish(rviz_marker);
}
