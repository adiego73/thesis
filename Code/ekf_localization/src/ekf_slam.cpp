#include "ekf_localization/ekf_slam.hpp"
#include <memory>
#include <octomap_msgs/conversions.h>

EKFSLAM::EKFSLAM(int robot_state_length, const std::map<std::string, std::string> &params) :
    m_current_state({Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}),
    m_map_manager(params.at("map_path"), params.at("octomap_path")),
    m_laser_range(),
    EKF(robot_state_length)
{
    // initialize poles and other known stuff.
    init(params);

    // register callback for the prediction step.
    m_s_drone_odom = m_nh.subscribe<nav_msgs::Odometry>(params.at("odometry_topic"), 10, &EKFSLAM::updateOdometry, this);

    if (params.at("enable_poles") == "true")
    {
        // register callback for correction when I see a pole.
        m_s_pole_landmark = m_nh.subscribe<poles_vision::RangeAndBearingPole>(params.at("pole_landmark_topic"), 10, &EKFSLAM::correctWithPole, this);
    }

    if (params.at("enable_markers") == "true")
    {
        // register callback for correction when I see a marker
        m_s_marker_landmark = m_nh.subscribe<ekf_localization::QRCodeStamped>(params.at("marker_landmark_topic"), 10, &EKFSLAM::correctWithMarker, this);
    }

    // setup octomap subscriber
    m_s_octomap = m_nh.subscribe<octomap_msgs::Octomap>(params.at("octomap_topic"), 10, &EKFSLAM::updateOctomap, this);

    if (params.at("enable_laser") == "true") {
        // setup range sensor subscriber
        m_s_range_sensor = m_nh.subscribe<sensor_msgs::Range>(params.at("range_sensor_topic"), 10, &EKFSLAM::correctWithLaserRange, this);
    }

    // setup the localization publisher.
    m_p_localization = m_nh.advertise<nav_msgs::Odometry>(params.at("drone_odom_topic"), 100);

    // setup markers publisher.
    m_p_markers = m_nh.advertise<ekf_localization::QRCodeStampedArray>(params.at("markers_pose_topic"), 10);

    // setup drone state service
    m_ss_drone_state = m_nh.advertiseService("get_drone_state", &EKFSLAM::getDroneState, this);

    // setup marker state service
    m_ss_marker_state = m_nh.advertiseService("get_marker_state", &EKFSLAM::getMarkerState, this);

    // setup save map service
    m_ss_save_map = m_nh.advertiseService("save_map", &EKFSLAM::saveMap, this);

#ifdef DEBUG
    p_rviz_marker = m_nh.advertise<visualization_msgs::Marker>("/visualization/qr_marker_estimate", 10);
    p_marker_cov = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/qr_marker_w_cov", 10);
#endif
}

EKFSLAM::~EKFSLAM()
{
#ifdef DEBUG
    std::cout << ":: Robot state " << m_state.block<4, 1>(0, 0).transpose() << std::endl;

    for (const auto &landmark_ptr : m_map_manager.getAll<Marker>())
    {
        std::cout << "::: MARKER "
                  << " " << landmark_ptr->ID << " at ";
        std::cout << landmark_ptr->getState(0) << " " << landmark_ptr->getState(1) << " " << landmark_ptr->getState(2) << " " << landmark_ptr->getState(3) << " " << landmark_ptr->getState(4) << " " << landmark_ptr->getState(5) << std::endl;
    }
//    std::cout << ":: Covariance matrix: \n"
//              << _covariance << std::endl;

//    std::cout << "Saving markers map...." << std::endl;
//    m_map_manager.save();
#endif

    m_s_drone_odom.shutdown();
    m_s_marker_landmark.shutdown();
    m_s_pole_landmark.shutdown();
    m_s_octomap.shutdown();
    m_s_range_sensor.shutdown();
    m_ss_marker_state.shutdown();
    m_ss_drone_state.shutdown();
    m_ss_save_map.shutdown();
}

void
EKFSLAM::updateOdometry(const nav_msgs::OdometryConstPtr &odometry_msg)
{
    m_last_drone_odom.header.stamp = ros::Time::now();
    m_last_drone_odom.pose.pose.position.x = odometry_msg->pose.pose.position.x;
    m_last_drone_odom.pose.pose.position.y = odometry_msg->pose.pose.position.y;
    m_last_drone_odom.pose.pose.position.z = odometry_msg->pose.pose.position.z;

    m_last_drone_odom.pose.pose.orientation.x = odometry_msg->pose.pose.orientation.x;
    m_last_drone_odom.pose.pose.orientation.y = odometry_msg->pose.pose.orientation.y;
    m_last_drone_odom.pose.pose.orientation.z = odometry_msg->pose.pose.orientation.z;
    m_last_drone_odom.pose.pose.orientation.w = odometry_msg->pose.pose.orientation.w;

    m_last_drone_odom.twist.twist.linear.x = odometry_msg->twist.twist.linear.x;
    m_last_drone_odom.twist.twist.linear.y = odometry_msg->twist.twist.linear.y;
    m_last_drone_odom.twist.twist.linear.z = odometry_msg->twist.twist.linear.z;

    m_last_drone_odom.twist.twist.angular.x = odometry_msg->twist.twist.angular.x;
    m_last_drone_odom.twist.twist.angular.y = odometry_msg->twist.twist.angular.y;
    m_last_drone_odom.twist.twist.angular.z = odometry_msg->twist.twist.angular.z;

    predict();
}

void
EKFSLAM::g(const Eigen::VectorXd &control)
{
    double vx = control(0), vy = control(1), vz = control(2);
    double omega_z = control(5);
    double roll = control(6), pitch = control(7);

    Eigen::Matrix4d T;
    // body to world transform.
    get_homogeneous_transformation(0, 0, 0, roll, pitch, m_state(3), T);

    Eigen::Vector3d linear_velocity;
    linear_velocity << vx, vy, vz;

    m_global_velocity = T.block<3, 3>(0, 0) * linear_velocity;

    m_state(0) += m_delta_t * m_global_velocity(0);
    m_state(1) += m_delta_t * m_global_velocity(1);
    m_state(2) += m_delta_t * m_global_velocity(2);

    m_state(3) += m_delta_t * omega_z;

    m_current_state.euler(0) = control(6);
    m_current_state.euler(1) = control(7);
    // update drone current state, from state vector.
    updateCurrentState();
}

Eigen::MatrixXd
EKFSLAM::G(const Eigen::VectorXd &control) const
{
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(m_state.size(), m_state.size());

    double vx = control(0), vy = control(1), vz = control(2), roll = control(6), pitch = control(7), yaw = control(8);

    G(0, 3) = -m_delta_t * (vy * (std::cos(roll) * std::cos(yaw) + std::sin(pitch) * std::sin(roll) * std::sin(yaw)) - vz * (std::cos(yaw) * std::sin(roll) - std::cos(roll) * std::sin(pitch) * std::sin(yaw)) + vx * std::cos(pitch) * std::sin(yaw));
    G(1, 3) = m_delta_t * (vz * (std::sin(roll) * std::sin(yaw) + std::cos(roll) * std::cos(yaw) * std::sin(pitch)) - vy * (std::cos(roll) * std::sin(yaw) - std::cos(yaw) * std::sin(pitch) * std::sin(roll)) + vx * std::cos(pitch) * std::cos(yaw));

    return G;
}

Eigen::VectorXd
EKFSLAM::h(const Landmark &landmark) const
{
    return landmark.getObservationModel(m_current_state);
}

Eigen::MatrixXd
EKFSLAM::H(const Landmark &landmark) const
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(landmark.getStateSize(), m_state.size());

    // jacobian wrt the robot state
    // the way the seen landmark affects the current robot's position
    H.block(0, 0, landmark.getStateSize(), 4) << landmark.getJacobianWrtDroneState(m_current_state, 4);

    // if the landmark is a marker and is in the state vector:
    if (LandmarkType::MARKER == landmark.TYPE
        && m_marker_indices.find(landmark.ID) != m_marker_indices.end())
    {

        int marker_index = m_marker_indices.at(landmark.ID);
        // jacobian wrt the marker state (x, y, z, r, p, y)
        // the way the seen landmark affects its own position.
        H.block(0, marker_index, landmark.getStateSize(), landmark.getStateSize()) << landmark.getJacobianWrtLandmarkState(m_current_state);
    }
    return H;
}

void
EKFSLAM::predict()
{
    if (m_first_message) {
        m_time = ros::Time::now();
        m_first_message = false;
    }
    // update delta t
    const ros::Time &current_time = ros::Time::now();
    m_delta_t = (current_time - m_time).toSec();
    m_time = current_time;

    Eigen::VectorXd control = Eigen::VectorXd::Zero(9);

    auto drone_linear_vel = m_last_drone_odom.twist.twist.linear;
    auto drone_angular_vel = m_last_drone_odom.twist.twist.angular;
    auto drone_orientation = m_last_drone_odom.pose.pose.orientation;

    // find drone's orientation as euler angles.
    tf2::Quaternion q_drone_orientation(drone_orientation.x, drone_orientation.y, drone_orientation.z, drone_orientation.w);
    Eigen::Vector3d euler;
    get_euler_from_quaternion(q_drone_orientation, euler);

    // build control vector [ vx vy vz omega_x omega_y omega_z roll pitch yaw]
    control << drone_linear_vel.x, drone_linear_vel.y, drone_linear_vel.z, drone_angular_vel.x, drone_angular_vel.y, drone_angular_vel.z, wrapToPi(euler(0)), wrapToPi(euler(1)), wrapToPi(euler(2));

    // do the prediction
    prediction(control);
}

Eigen::MatrixXd
EKFSLAM::getControlCovariance() const
{
    Eigen::Matrix4d U = Eigen::Matrix4d::Zero();
    U(0, 0) = U(1, 1) = U(2, 2) = m_avg_linear_vel;
    U(3, 3) = m_avg_angular_vel;

    double roll = m_current_state.euler(0), pitch = m_current_state.euler(1), yaw = m_current_state.euler(2);
    double x = m_current_state.position(0), y = m_current_state.position(1), z = m_current_state.position(2);

    double cr = std::cos(roll), cp = std::cos(pitch), cy = std::cos(yaw);
    double sr = std::sin(roll), sp = std::sin(pitch), sy = std::sin(yaw);

    double crcy = cr * cy;
    double cpsr = cp * sr;
    double crsy = cr * sy;
    double cpsy = cp * sy;
    double cysr = cy * sr;
    double cpcr = cp * cr;
    double cpcy = cp * cy;
    double srsy = sr * sy;
    double cyspsr = cy * sp * sr;
    double crcysp = cr * cy * sp;
    double crspsy = cr * sp * sy;
    double spsrsy = sp * sr * sy;
    double crsy_cyspsr = crsy - cyspsr;
    double srsy_crcysp = srsy + crcysp;
    double cysr_crspsy = cysr - crspsy;
    double crcy_spsrsy = crcy + spsrsy;

    double N00 = yaw / 2 + (cpcy) / 2;
    double N01 = (cyspsr) / 2 - (crsy) / 2;
    double N02 = (srsy) / 2 + (crcysp) / 2;
    double N03 = x / 2 - (y * (crcy_spsrsy)) / 2 + (z * (cysr_crspsy)) / 2 - (x * cpsy) / 2;
    double N10 = (cpsy) / 2;
    double N11 = yaw / 2 + (crcy) / 2 + (spsrsy) / 2;
    double N12 = (crspsy) / 2 - (cysr) / 2;
    double N13 = y / 2 - (y * (crsy_cyspsr)) / 2 + (z * (srsy_crcysp)) / 2 + (x * cpcy) / 2;
    double N20 = -sp / 2;
    double N21 = cpsr / 2;
    double N22 = yaw / 2 + (cpcr) / 2;
    double N23 = z / 2;

    double N30 = 0, N31 = 0, N32 = 0;
    double N33 = 0.5;

    Eigen::Matrix4d N;
    N << N00, N01, N02, N03, N10, N11, N12, N13, N20, N21, N22, N23, N30, N31, N32, N33;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m_state.size(), m_state.size());
    R.block<4, 4>(0, 0) = N * U * N.transpose();

    return R;
}

Eigen::MatrixXd
EKFSLAM::getObservationCovariance(const LandmarkType &type) const
{
    if (LandmarkType::POLE == type)
    {
        return m_pole_Q;
    }
    else if (LandmarkType::MARKER == type)
    {
        return m_marker_Q;
    }
    else {
        return m_range_Q;
    }
}

void
EKFSLAM::correctWithMarker(const ekf_localization::QRCodeStampedConstPtr &marker_msg)
{
//    ROS_INFO("Correct with marker %d", marker_msg->marker.code);
    int marker_id = marker_msg->marker.code;
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(6);

    auto m_position = marker_msg->marker.pose.position;
    auto m_orientation = marker_msg->marker.pose.orientation;

    double m_roll, m_pitch, m_yaw;
    tf::Quaternion m_quaternion(m_orientation.x, m_orientation.y, m_orientation.z, m_orientation.w);
    tf::Matrix3x3(m_quaternion).getRPY(m_roll, m_pitch, m_yaw);

    observation << m_position.x, m_position.y, m_position.z, m_roll, m_pitch, m_yaw;

    if (!m_map_manager.exists<Marker>(marker_id))
    {
        // add marker, update state vector and covariance matrix
        addLandmarkToMap(observation, marker_id);
    }

    try
    {
        auto &marker = m_map_manager.getLandmark<Marker>(marker_id);

        // do not correct if the marker was not saved before.
        correction(observation, marker);

        // update marker state from state vector, if needed
        updateMarkerState(marker);
    }
    catch (std::out_of_range &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void
EKFSLAM::addLandmarkToMap(const Eigen::VectorXd &observation, int landmark_id)
{
    int start_index = m_state.size();
    m_marker_indices.insert({landmark_id, start_index});

    // add landmark to the map
    auto marker = m_map_manager.addLandmark<Marker>(std::make_unique<Marker>(landmark_id));

    marker.getInverseObservationModel(m_current_state, observation);

    // covariance matrix of robot state.
    Eigen::Matrix4d Pxx = m_covariance.block<4, 4>(0, 0);

    // jacobian of the inverse observation model wrt the robot variables.
    Eigen::MatrixXd J_inv_h_x = marker.getInverseJacobianWrtDroneState(m_current_state, 4);

    // Jacobian of the inverse observation model wrt the observation variables.
    Eigen::MatrixXd J_inv_h_z = marker.getInverseJacobianWrtLandmarkState(m_current_state);

    /*
     * Peter Corke - SLAM EKF: the insertion Jacobian
     * https://petercorke.com/robotics/ekf-covariance-matrix-update-for-a-new-landmark/
     +-----+-----+
     | Pxx | Pmx |
     +-----+-----+
     | Pxm | Pmm |
     +-----+-----+
     */

    long old_size = m_covariance.cols();
    long new_size = m_covariance.cols() + 6;
    Eigen::MatrixXd Sigma(m_covariance);
    m_covariance.resize(new_size, new_size);

    Eigen::MatrixXd Yx = Eigen::MatrixXd::Identity(new_size, old_size);
    Yx.bottomLeftCorner(J_inv_h_x.rows(), J_inv_h_x.cols()) = J_inv_h_x;

    Eigen::MatrixXd Yz = Eigen::MatrixXd::Zero(new_size, J_inv_h_z.cols());
    Yz.bottomRightCorner(J_inv_h_z.rows(), J_inv_h_z.cols()) = J_inv_h_z;

    // update covariance matrix
    m_covariance = Yx * Sigma * Yx.transpose() + Yz * getObservationCovariance(LandmarkType::MARKER) * Yz.transpose();
    // enlarge state vector and save the new marker
    m_state.conservativeResize(m_state.size() + 6);
    m_state.block<6, 1>(start_index, 0) << marker.getState();

    ROS_INFO("Marker %d added at %f %f %f %f %f %f", landmark_id, marker.getState(0), marker.getState(1), marker.getState(2), marker.getState(3), marker.getState(4), marker.getState(5));
}

void
EKFSLAM::publishState() const
{
    // publish the drone state.
    nav_msgs::Odometry message;
    std::string frame_id = "map";

    message.header.stamp = ros::Time::now();
    message.header.frame_id = frame_id;
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = m_current_state.position(0);
    message.pose.pose.position.y = m_current_state.position(1);
    message.pose.pose.position.z = m_current_state.position(2);

    message.twist.twist.linear.x = m_global_velocity(0);
    message.twist.twist.linear.y = m_global_velocity(1);
    message.twist.twist.linear.z = m_global_velocity(2);

    tf2::Quaternion orientation;
    orientation.setRPY(m_current_state.euler(0), m_current_state.euler(1), m_current_state.euler(2));

    message.pose.pose.orientation = tf2::toMsg(orientation);

    for (int c = 0; c < 6; c++)
    {
        for (int r = 0; r < 6; r++)
        {
            // we dont have estimations for roll and pitch, so we set to 0 the covariance value
            if (r == 3 || r == 4 || c == 3 || c == 4)
            {
                message.pose.covariance[c + r * 6] = 0.0;
            }
            else
            {
                int column = c == 5 ? 3 : c;
                int row = r == 5 ? 3 : r;
                message.pose.covariance[c + r * 6] = m_covariance(row, column);
            }
        }
    }

    m_p_localization.publish(message);

    // publish markers state
    ekf_localization::QRCodeStampedArray markers_msg;
    markers_msg.header.frame_id = frame_id;
    markers_msg.header.stamp = ros::Time::now();

    for (const auto &marker_ptr : m_map_manager.getAll<Marker>())
    {
        ekf_localization::QRCodePose marker_pose;
        marker_pose.code = marker_ptr->ID;
        marker_pose.pose.position.x = marker_ptr->getState(0);
        marker_pose.pose.position.y = marker_ptr->getState(1);
        marker_pose.pose.position.z = marker_ptr->getState(2);

        tf2::Quaternion q;
        q.setRPY(marker_ptr->getState(3), marker_ptr->getState(4), marker_ptr->getState(5));
        marker_pose.pose.orientation = tf2::toMsg(q);

        markers_msg.markers.emplace_back(marker_pose);
    }

    m_p_markers.publish(markers_msg);
}

void
EKFSLAM::correctWithLaserRange(const sensor_msgs::RangeConstPtr &range_msg)
{
    Eigen::Matrix4d laser_transform;
    get_homogeneous_transformation(0.05, 0, -0.05, 0,0,0, laser_transform);

    double x = m_current_state.position(0),
           y = m_current_state.position(1),
           z = m_current_state.position(2),
           roll = m_current_state.euler(0),
           pitch = m_current_state.euler(1),
           yaw = m_current_state.euler(2);

    Eigen::Matrix4d robot_transform;
    get_homogeneous_transformation(x,y,z, roll, pitch, yaw, robot_transform);

    Eigen::Matrix4d laser = laser_transform * robot_transform;

    Eigen::Vector3d voxel;
    if(m_map_manager.getVoxelFrom(laser.block<3,1>(0, 3), voxel))
    {
        Eigen::VectorXd observation(1);
        observation << range_msg->range + voxel(2);

        correction(observation, m_laser_range);

        updateCurrentState();
    }
}

void
EKFSLAM::correctWithPole(const poles_vision::RangeAndBearingPoleConstPtr &pole_msg)
{
//    ROS_INFO("Correct with pole %d", pole_msg->id);
    try
    {
        const auto &pole = m_map_manager.getLandmark<Pole>(pole_msg->id);

        Eigen::VectorXd observations = Eigen::VectorXd::Zero(6);

        // elevation is not considered
        observations << pole_msg->distance, pole_msg->azimuth, pole_msg->elevation, pole_msg->trust[2], pole_msg->trust[0], pole_msg->trust[1];

        // do the correction when I see a marker
        correction(observations, pole);

        // update the drone current state using the state vector.
        updateCurrentState();
    }
    catch (std::out_of_range &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

Eigen::VectorXd
EKFSLAM::innovation(const Eigen::VectorXd &observations, const Landmark &landmark) const
{
    Eigen::VectorXd innovation;

    if (LandmarkType::POLE == landmark.TYPE)
    {
        Eigen::Vector3d obs_hat = landmark.getObservationModel(m_current_state);
        Eigen::Vector3d trust = observations.block<3, 1>(3, 0);
        innovation = observations.block<3, 1>(0, 0) - obs_hat;

        innovation(0) = trust(0) ? innovation(0) : 0;// distance
        innovation(1) = trust(1) ? innovation(1) : 0;// azimuth
        innovation(2) = trust(2) ? innovation(2) : 0;// elevation

        wrapToPiInPlace(innovation(1));
        wrapToPiInPlace(innovation(2));
    }
    else if (LandmarkType::MARKER == landmark.TYPE)
    {
        innovation = EKF::innovation(observations, landmark);
        wrapToPiInPlace(innovation(3));
        wrapToPiInPlace(innovation(4));
        wrapToPiInPlace(innovation(5));
    }
    else {
        innovation = EKF::innovation(observations, landmark);
    }

    return innovation;
}

void
EKFSLAM::updateOctomap(const octomap_msgs::OctomapConstPtr &octomap_msg)
{
    auto map = dynamic_cast<octomap::ColorOcTree *>(octomap_msgs::msgToMap(*octomap_msg));
    m_map_manager.updateOctomap(map);
}

constexpr double
EKFSLAM::chiSquareLimit(const LandmarkType &type) const
{
    if (LandmarkType::POLE == type) {
        // chi2(3, 1-0.05)
        return 7.8147;
    }
    else if (LandmarkType::MARKER == type)
    {
        // chi2(6, 1-0.05)
        return 12.5916;
    }
    else {
        // chi2(1, 1-0.05)
        return 3.841;
    }
}

void
EKFSLAM::run()
{
    //    predict();

    publishState();
#ifdef DEBUG
    rvizDrawMarkers();
#endif
}

void
EKFSLAM::init(const std::map<std::string, std::string> &params)
{
    // initialize robot state pose.
    m_nh.param("/ekf_localization_node/initial_position_x", m_current_state.position(0), 0.0);
    m_nh.param("/ekf_localization_node/initial_position_y", m_current_state.position(1), 0.0);
    m_nh.param("/ekf_localization_node/initial_position_z", m_current_state.position(2), 0.0);
    m_nh.param("/ekf_localization_node/initial_position_yaw", m_current_state.euler(2), 0.0);

    m_nh.param("/ekf_localization_node/avg_linear_vel", m_avg_linear_vel, 0.1);    // 0.1 m/s
    m_nh.param("/ekf_localization_node/avg_angular_vel", m_avg_angular_vel, 0.087);// 5 deg/s

    // load map from file.
    m_map_manager.load();

    std::vector<double> poles_noise_covariance;
    if (m_nh.getParam("/ekf_localization_node/poles_noise_covariance", poles_noise_covariance))
    {
        ROS_INFO("Setting up Poles noise covariance matrix");
        int row = 0;
        int column = 0;
        for (; row < m_pole_Q.rows(); row++)
        {
            m_pole_Q.row(row) << poles_noise_covariance[column], poles_noise_covariance[column + 1], poles_noise_covariance[column + 2];
            column += 3;
        }
    }

    std::vector<double> markers_noise_covariance;
    if (m_nh.getParam("/ekf_localization_node/markers_noise_covariance", markers_noise_covariance))
    {
        ROS_INFO("Setting up Markers noise covariance matrix");
        int row = 0;
        int column = 0;
        for (; row < m_marker_Q.rows(); row++)
        {
            m_marker_Q.row(row).block<1, 3>(0, 0) << markers_noise_covariance[column], markers_noise_covariance[column + 1], markers_noise_covariance[column + 2];
            m_marker_Q.row(row).block<1, 3>(0, 3) << markers_noise_covariance[column + 3], markers_noise_covariance[column + 4], markers_noise_covariance[column + 5];
            column += 6;
        }
    }

    double range_noise_covariance;
    if (m_nh.getParam("/ekf_localization_node/range_noise_covariance", range_noise_covariance))
    {
        ROS_INFO("Setting up Range noise covariance matrix");
        m_range_Q << range_noise_covariance;
    }

    m_last_drone_odom.pose.pose.position.x = m_state(0) = m_current_state.position(0);
    m_last_drone_odom.pose.pose.position.y = m_state(1) = m_current_state.position(1);
    m_last_drone_odom.pose.pose.position.z = m_state(2) = m_current_state.position(2);

    m_last_drone_odom.pose.pose.orientation.x = 0;
    m_last_drone_odom.pose.pose.orientation.y = 0;
    m_last_drone_odom.pose.pose.orientation.z = 0;
    m_last_drone_odom.pose.pose.orientation.w = 1;

    m_last_drone_odom.twist.twist.linear.x = 0;
    m_last_drone_odom.twist.twist.linear.y = 0;
    m_last_drone_odom.twist.twist.linear.z = 0;

    m_last_drone_odom.twist.twist.angular.x = 0;
    m_last_drone_odom.twist.twist.angular.y = 0;
    m_last_drone_odom.twist.twist.angular.z = 0;
}

bool
EKFSLAM::getMarkerState(ekf_localization::GetMarkerState::Request &req, ekf_localization::GetMarkerState::Response &res)
{
    try
    {
        const auto &marker = m_map_manager.getLandmark<Marker>(req.id);
        res.pose.position.x = marker.getState(0);
        res.pose.position.y = marker.getState(1);
        res.pose.position.z = marker.getState(2);

        tf2::Quaternion q;
        q.setRPY(marker.getState(3), marker.getState(4), marker.getState(5));
        res.pose.orientation = tf2::toMsg(q);

        return true;
    }
    catch (std::out_of_range &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
}

bool
EKFSLAM::getDroneState(ekf_localization::GetDroneState::Request &req, ekf_localization::GetDroneState::Response &res)
{
    res.pose.position.x = m_current_state.position(0);
    res.pose.position.y = m_current_state.position(1);
    res.pose.position.z = m_current_state.position(2);

    tf2::Quaternion q;
    q.setRPY(m_current_state.euler(0), m_current_state.euler(1), m_current_state.euler(2));
    res.pose.orientation = tf2::toMsg(q);

    return true;
}

bool
EKFSLAM::saveMap(ekf_localization::SaveMap::Request &req, ekf_localization::SaveMap::Response &res)
{
    ROS_INFO("Saving map...");
    m_map_manager.save();
    return true;
}

void
EKFSLAM::updateCurrentState()
{
    wrapToPiInPlace(m_state(3));

    m_current_state.position(0) = m_state(0);
    m_current_state.position(1) = m_state(1);
    m_current_state.position(2) = m_state(2);
    m_current_state.euler(2) = m_state(3);
}

void
EKFSLAM::updateMarkerState(Landmark &landmark)
{
    if (m_marker_indices.find(landmark.ID) != m_marker_indices.end())
    {
        int marker_index = m_marker_indices.at(landmark.ID);

        // update angles from corrected state
        wrapToPiInPlace(m_state(marker_index + 3));
        wrapToPiInPlace(m_state(marker_index + 4));
        wrapToPiInPlace(m_state(marker_index + 5));

        landmark.setState(m_state.block<6, 1>(marker_index, 0));
    }
}

#ifdef DEBUG

void
EKFSLAM::rvizDrawMarkers() const
{
    for (const auto &pair : m_marker_indices)
    {
        int id = pair.first;
        int index = pair.second;

        geometry_msgs::PoseWithCovarianceStamped marker_cov;
        visualization_msgs::Marker rviz_marker;
        marker_cov.header.stamp = rviz_marker.header.stamp = ros::Time::now();
        marker_cov.header.frame_id = rviz_marker.header.frame_id = "map";

        Eigen::Vector3d position = m_state.block(index, 0, 3, 1);
        Eigen::Vector3d euler = m_state.block(index + 3, 0, 3, 1);

        tf2::Quaternion q;
        q.setRPY(euler(0), euler(1), euler(2));

        rviz_marker.ns = "qr_marker_estimated";
        rviz_marker.id = id;

        rviz_marker.type = visualization_msgs::Marker::CUBE;
        rviz_marker.action = visualization_msgs::Marker::ADD;

        rviz_marker.pose.position.x = position(0);
        rviz_marker.pose.position.y = position(1);
        rviz_marker.pose.position.z = position(2);
        rviz_marker.pose.orientation = tf2::toMsg(q);

        marker_cov.pose.pose = rviz_marker.pose;

        Eigen::MatrixXd covariance = m_covariance.block<6, 6>(index, index);

        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                marker_cov.pose.covariance[c + r * 6] = covariance(r, c);
            }
        }

        rviz_marker.scale.x = 0.2;
        rviz_marker.scale.y = 0.2;
        rviz_marker.scale.z = 0.01;

        rviz_marker.color.a = 1.0;
        rviz_marker.color.r = 1.0;
        rviz_marker.color.g = 1.0;
        rviz_marker.color.b = 0.0;

        rviz_marker.lifetime = ros::Duration();

        p_rviz_marker.publish(rviz_marker);
        p_marker_cov.publish(marker_cov);
    }
}

#endif