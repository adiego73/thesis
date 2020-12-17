//
// Created by diego on 5/1/20.
//

#ifndef EKF_LOCALIZATION_EKF_SLAM_HPP
#define EKF_LOCALIZATION_EKF_SLAM_HPP

#include <cmath>
#include <memory>
#include <random>

#include "commons.hpp"
#include "ekf.hpp"
#include "map_manager.hpp"
#include "marker.hpp"
#include "pole.hpp"
#include "range.hpp"

class EKFSLAM : public EKF
{
private:
    ros::NodeHandle m_nh;

    bool m_first_message = true;
    ros::Time m_time = ros::Time::now();

    ros::Subscriber m_s_drone_odom;
    ros::Subscriber m_s_pole_landmark;
    ros::Subscriber m_s_marker_landmark;
    ros::Subscriber m_s_octomap;
    ros::Subscriber m_s_range_sensor;

    ros::Publisher m_p_localization;
    ros::Publisher m_p_markers;

    ros::ServiceServer m_ss_drone_state;
    ros::ServiceServer m_ss_marker_state;
    ros::ServiceServer m_ss_save_map;

    std::map<int, int> m_marker_indices;

    double m_delta_t = 0;

    DroneState m_current_state;

    double m_avg_linear_vel = 0;
    double m_avg_angular_vel = 0;

    Eigen::Vector3d m_global_velocity;
    nav_msgs::Odometry m_last_drone_odom;

    Eigen::Matrix3d m_pole_Q = Eigen::Matrix3d::Zero();
    Eigen::MatrixXd m_marker_Q = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd m_range_Q = Eigen::MatrixXd::Zero(1, 1);

    MapManager m_map_manager;

    Range m_laser_range;

    void
    init(const std::map<std::string, std::string> &params);

    void
    correctWithMarker(const ekf_localization::QRCodeStampedConstPtr &marker_msg);

    void
    correctWithPole(const poles_vision::RangeAndBearingPoleConstPtr &pole_msg);

    void
    correctWithLaserRange(const sensor_msgs::RangeConstPtr &range_msg);

    void
    updateOdometry(const nav_msgs::OdometryConstPtr &odometry_msg);

    void
    addLandmarkToMap(const Eigen::VectorXd &observation, int landmark_id);

    void
    predict();

    bool
    getMarkerState(ekf_localization::GetMarkerState::Request &req, ekf_localization::GetMarkerState::Response &res);

    bool
    getDroneState(ekf_localization::GetDroneState::Request &req, ekf_localization::GetDroneState::Response &res);

    bool
    saveMap(ekf_localization::SaveMap::Request &req, ekf_localization::SaveMap::Response &res);

    void
    updateCurrentState();

    void
    updateMarkerState(Landmark &landmark);

    void
    updateOctomap(const octomap_msgs::OctomapConstPtr &octomap_msg);

#ifdef DEBUG
    ros::Publisher p_rviz_marker;
    ros::Publisher p_marker_cov;

    void
    rvizDrawMarkers() const;

#endif

public:
    explicit EKFSLAM(int robot_state_length = 4, const std::map<std::string, std::string> &params = {});

    void
    publishState() const;

    void
    run();

    ~EKFSLAM() override;

    // remove copy and move constructors, and copy and move = operator, since all of them make no sense.
    EKFSLAM(const EKFSLAM &) = delete;

    EKFSLAM(EKFSLAM &&) = delete;

    EKFSLAM &
    operator=(const EKFSLAM &) = delete;

    EKFSLAM &
    operator=(const EKFSLAM &&) = delete;

protected:
    void
    g(const Eigen::VectorXd &control) override;

    Eigen::VectorXd
    h(const Landmark &landmark) const override;

    Eigen::MatrixXd
    H(const Landmark &landmark) const override;

    Eigen::MatrixXd
    G(const Eigen::VectorXd &control) const override;

    constexpr double
    chiSquareLimit(const LandmarkType &type) const override;

    Eigen::MatrixXd
    getControlCovariance() const override;

    Eigen::MatrixXd
    getObservationCovariance(const LandmarkType &type) const override;

    Eigen::VectorXd
    innovation(const Eigen::VectorXd &observations, const Landmark &landmark) const override;
};

#endif//EKF_LOCALIZATION_EKF_SLAM_HPP
