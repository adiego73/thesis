#include "ekf_localization/commons.hpp"

void
publishStringStamped(const std_msgs::StringConstPtr &message, const ros::Publisher &p);

void
publishQRCodeStamped(const geometry_msgs::PoseStampedConstPtr &pose, const ekf_localization::StringStampedConstPtr &data, const ros::Publisher &pub);

/**
 * Subscribes to /visp_auto_tracker/code_message and republish the message with a timestamp.
 * This is useful to get a bunch of messages related to QR codes.
 */
int
main(int argc, char **argv)
{
    ros::init(argc, argv, "qr_code_node");

    ROS_INFO(":: QR CODE node instantiated ::");

    ros::NodeHandle node("~");

    ros::Publisher p_transform = node.advertise<ekf_localization::QRCodeStamped>("/visp_auto_tracker/stamped_object_position", 10);
    ros::Publisher p_stamped_code = node.advertise<ekf_localization::StringStamped>("/visp_auto_tracker/stamped_message", 10);

    const std::string position_topic = "/visp_auto_tracker/object_position";

    ros::Subscriber s_qr_code = node.subscribe<std_msgs::String>("/visp_auto_tracker/code_message", 10, boost::bind(&publishStringStamped, _1, std::ref(p_stamped_code)));

    message_filters::Subscriber<ekf_localization::StringStamped> s_marker_data(node, "/visp_auto_tracker/stamped_message", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> s_marker_pose(node, position_topic, 10);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), s_marker_pose, s_marker_data);

    sync.registerCallback(boost::bind(&publishQRCodeStamped, _1, _2, std::ref(p_transform)));

    ros::spin();
}

/**
 * Publishes the QR data as a stamped message type.
 * @param message
 * @param p
 */
void
publishStringStamped(const std_msgs::StringConstPtr &message, const ros::Publisher &p)
{
    // publish stamped message only if data is not empty.
    if (!message->data.empty())
    {
        ekf_localization::StringStamped stamped;
        stamped.header.stamp = ros::Time::now();
        stamped.data = message->data;
        p.publish(stamped);
    }
}

/**
 * Publish QR pose and code/data, all together.
 * @param pose
 * @param data
 * @param pub
 */
void
publishQRCodeStamped(const geometry_msgs::PoseStampedConstPtr &pose, const ekf_localization::StringStampedConstPtr &data, const ros::Publisher &pub)
{
    ekf_localization::QRCodeStamped qr_msg;
    qr_msg.header.stamp = ros::Time::now();

    qr_msg.marker.pose = pose->pose;
    qr_msg.marker.data = data->data;

    std::string code = qr_msg.marker.data.substr(qr_msg.marker.data.find_last_of(' '));
    try
    {
        qr_msg.marker.code = std::stoi(code);
    }
    catch (std::exception &e)
    {
        ROS_WARN("%s", e.what());
        qr_msg.marker.code = -1;
    }

    pub.publish(qr_msg);
}