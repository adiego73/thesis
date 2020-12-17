#ifndef EKF_LOCALIZATION_COMMONS_HPP
#define EKF_LOCALIZATION_COMMONS_HPP

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Range.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/StdVector>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ekf_localization/QRCodeStamped.h"
#include "ekf_localization/QRCodeStampedArray.h"
#include "ekf_localization/StringStamped.h"
#include "poles_vision/RangeAndBearingPole.h"

#include "ekf_localization/GetDroneState.h"
#include "ekf_localization/GetMarkerState.h"
#include "ekf_localization/SaveMap.h"

#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

struct DroneState
{
    Eigen::Vector3d position;
    Eigen::Vector3d euler;
};

enum class LandmarkType : int8_t
{
    MARKER = 0,
    POLE = 1,
    RANGE = 2
};

using SyncPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, ekf_localization::StringStamped>;

void
get_homogeneous_transformation(double x, double y, double z, double roll, double pitch, double yaw, Eigen::Matrix4d &out);
void
get_euler_from_quaternion(tf2::Quaternion q, Eigen::Vector3d &out);
void
get_quaternion_from_matrix(const Eigen::Matrix3d &R, tf2::Quaternion &out);
void
get_euler_from_matrix(const Eigen::Matrix4d &R, Eigen::Vector3d &out);

void
wrapTo2PiInPlace(double &angle);
void
wrapToPiInPlace(double &angle);
double
wrapToPi(double angle);
double
radToDeg(double angle);

std::string
get_landmark_name(const LandmarkType& type);

#endif //EKF_LOCALIZATION_COMMONS_HPP
