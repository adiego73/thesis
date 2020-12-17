#include "ekf_localization/marker.hpp"

Marker::Marker(int id) :
    Landmark(id, LandmarkType::MARKER, 6)
{
}

Marker::Marker(int id, double x, double y, double z, double roll, double pitch, double yaw) :
    Landmark(id, LandmarkType::MARKER, 6)
{
    m_state << x, y, z, roll, pitch, yaw;
}

Eigen::VectorXd
Marker::getObservationModel(const DroneState &drone_state) const
{
    Eigen::Matrix4d Tm;
    get_homogeneous_transformation(m_state(0), m_state(1), m_state(2), m_state(3), m_state(4), m_state(5), Tm);

    double x = drone_state.position(0), y = drone_state.position(1), z = drone_state.position(2);

    Eigen::Matrix4d Tr;
    get_homogeneous_transformation(x, y, z, drone_state.euler(0), drone_state.euler(1), drone_state.euler(2), Tr);

    Eigen::Matrix4d Tc;
    get_homogeneous_transformation(0.0, 0.0, 0.0, -M_PI, 0.0, -M_PI_2, Tc);

    Eigen::Matrix4d T = Tr * Tc;

    Eigen::Matrix4d marker_local_pos = T.inverse() * Tm;

    Eigen::Vector3d euler_m;
    get_euler_from_matrix(marker_local_pos, euler_m);

    Eigen::VectorXd h_hat(state_size);
    h_hat << marker_local_pos(0, 3), marker_local_pos(1, 3), marker_local_pos(2, 3), wrapToPi(euler_m(0)), wrapToPi(euler_m(1)), wrapToPi(euler_m(2));

    return h_hat;
}

Eigen::MatrixXd
Marker::getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const
{
    Eigen::Vector3d m_position = m_state.block<3, 1>(0, 0);
    Eigen::Vector3d m_orientation = m_state.block<3, 1>(3, 0);

    // robot
    double cr = std::cos(drone_state.euler(0)), cp = std::cos(drone_state.euler(1)), cy = std::cos(drone_state.euler(2));
    double sr = std::sin(drone_state.euler(0)), sp = std::sin(drone_state.euler(1)), sy = std::sin(drone_state.euler(2));
    double x0 = drone_state.position(0), y0 = drone_state.position(1);
    // marker
    double xm = m_position(0), ym = m_position(1);
    double crm = std::cos(m_orientation(0)), cpm = std::cos(m_orientation(1)), cym = std::cos(m_orientation(2));
    double srm = std::sin(m_orientation(0)), spm = std::sin(m_orientation(1)), sym = std::sin(m_orientation(2));

    Eigen::MatrixXd Jacobian(state_size, drone_state_size);

    double H00 = cy * sp * sr - cr * sy,
        H01 = cr * cy + sp * sr * sy,
        H02 = cp * sr,
        H03 = xm * (cr * cy + sp * sr * sy) + ym * (cr * sy - cy * sp * sr) - x0 * cr * cy - y0 * cr * sy + y0 * cy * sp * sr - x0 * sp * sr * sy;
    double H10 = cp * cy,
        H11 = cp * sy,
        H12 = -sp,
        H13 = y0 * cp * cy - ym * cp * cy - x0 * cp * sy + xm * cp * sy;
    double H20 = sr * sy + cr * cy * sp,
        H21 = cr * sp * sy - cy * sr,
        H22 = cp * cr,
        H23 = x0 * cy * sr - ym * (sr * sy + cr * cy * sp) - xm * (cy * sr - cr * sp * sy) + y0 * sr * sy + y0 * cr * cy * sp - x0 * cr * sp * sy;
    double H30 = 0,
        H31 = 0,
        H32 = 0,
        H33 = ((((sr * sy + cr * cy * sp) * (crm * cym + spm * srm * sym) - (cy * sr - cr * sp * sy) * (crm * sym - cym * spm * srm)) / ((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm) -
        (((sr * sy + cr * cy * sp) * (cym * srm - crm * spm * sym) - (cy * sr - cr * sp * sy) * (srm * sym + crm * cym * spm)) * ((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm)) / std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) *
        std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) / (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2));
    double H40 = 0,
        H41 = 0,
        H42 = 0,
        H43 = ((std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)) *
        ((cpm * cym * (cy * sr - cr * sp * sy) + cpm * sym * (sr * sy + cr * cy * sp)) / std::sqrt(std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)) -
            ((2 * ((sr * sy + cr * cy * sp) * (cym * srm - crm * spm * sym) - (cy * sr - cr * sp * sy) * (srm * sym + crm * cym * spm)) * ((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm) +
                2 * ((sr * sy + cr * cy * sp) * (crm * cym + spm * srm * sym) - (cy * sr - cr * sp * sy) * (crm * sym - cym * spm * srm)) * ((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm)) * (cpm * sym * (cy * sr - cr * sp * sy) - cpm * cym * (sr * sy + cr * cy * sp) + cp * cr * spm)) /
                (2 * std::pow(std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2), 1.5)))) /
        (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2) + std::pow(cpm * sym * (cy * sr - cr * sp * sy) - cpm * cym * (sr * sy + cr * cy * sp) + cp * cr * spm, 2));
    double H50 = 0,
        H51 = 0,
        H52 = 0,
        H53 = -(((cp * cpm * cy * sym - cp * cpm * cym * sy) / (cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr) - ((cpm * cym * (cr * cy + sp * sr * sy) + cpm * sym * (cr * sy - cy * sp * sr)) * (sp * spm + cp * cpm * cy * cym + cp * cpm * sy * sym)) / std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)) *
        std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)) / (std::pow(sp * spm + cp * cpm * cy * cym + cp * cpm * sy * sym, 2) + std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2));

    // jacobian wrt the robot state
    // the way the seen landmark affects the current robot's position
    Jacobian << H00, H01, H02, H03, H10, H11, H12, H13, H20, H21, H22, H23, H30, H31, H32, H33, H40, H41, H42, H43, H50, H51, H52, H53;

    return Jacobian;
}

Eigen::MatrixXd
Marker::getJacobianWrtLandmarkState(const DroneState &drone_state) const
{
    Eigen::Vector3d m_position = m_state.block<3, 1>(0, 0);
    Eigen::Vector3d m_orientation = m_state.block<3, 1>(3, 0);

    // robot
    double cr = std::cos(drone_state.euler(0)), cp = std::cos(drone_state.euler(1)), cy = std::cos(drone_state.euler(2));
    double sr = std::sin(drone_state.euler(0)), sp = std::sin(drone_state.euler(1)), sy = std::sin(drone_state.euler(2));
    // marker
    double crm = std::cos(m_orientation(0)), cpm = std::cos(m_orientation(1)), cym = std::cos(m_orientation(2));
    double srm = std::sin(m_orientation(0)), spm = std::sin(m_orientation(1)), sym = std::sin(m_orientation(2));

    Eigen::MatrixXd Jacobian(state_size, state_size);

    double H00m = cr * sy - cy * sp * sr,
        H01m = -cr * cy - sp * sr * sy,
        H02m = -cp * sr,
        H03m = 0,
        H04m = 0,
        H05m = 0;
    double H10m = -cp * cy,
        H11m = -cp * sy,
        H12m = sp,
        H13m = 0,
        H14m = 0,
        H15m = 0;
    double H20m = -sr * sy - cr * cy * sp,
        H21m = cy * sr - cr * sp * sy,
        H22m = -cp * cr,
        H23m = 0,
        H24m = 0,
        H25m = 0;
    double H30m = 0,
        H31m = 0,
        H32m = 0,
        H33m = ((std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2) / std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + 1) * std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) /
        (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)), H34m =
        -(((cpm * srm * sym * (cy * sr - cr * sp * sy) - cpm * cym * srm * (sr * sy + cr * cy * sp) + cp * cr * spm * srm) / ((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm) +
            ((cpm * crm * sym * (cy * sr - cr * sp * sy) - cpm * crm * cym * (sr * sy + cr * cy * sp) + cp * cr * crm * spm) * ((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm)) / std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) *
            std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) / (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)), H35m =
        -((((sr * sy + cr * cy * sp) * (crm * cym + spm * srm * sym) - (cy * sr - cr * sp * sy) * (crm * sym - cym * spm * srm)) / ((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm) -
            (((sr * sy + cr * cy * sp) * (cym * srm - crm * spm * sym) - (cy * sr - cr * sp * sy) * (srm * sym + crm * cym * spm)) * ((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm)) / std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) *
            std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2)) / (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2));
    double H40m = 0,
        H41m = 0,
        H42m = 0,
        H43m = 0,
        H44m = -((std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)) *
        ((cym * spm * (sr * sy + cr * cy * sp) - spm * sym * (cy * sr - cr * sp * sy) + cp * cpm * cr) / std::sqrt(std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)) +
            ((2 * (cpm * crm * sym * (cy * sr - cr * sp * sy) - cpm * crm * cym * (sr * sy + cr * cy * sp) + cp * cr * crm * spm) * ((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm) -
                2 * (cpm * srm * sym * (cy * sr - cr * sp * sy) - cpm * cym * srm * (sr * sy + cr * cy * sp) + cp * cr * spm * srm) * ((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm)) * (cpm * sym * (cy * sr - cr * sp * sy) - cpm * cym * (sr * sy + cr * cy * sp) + cp * cr * spm)) /
                (2 * std::pow(std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2), 1.5)))) /
        (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2) + std::pow(cpm * sym * (cy * sr - cr * sp * sy) - cpm * cym * (sr * sy + cr * cy * sp) + cp * cr * spm, 2)), H45m =
        -((std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)) *
            ((cpm * cym * (cy * sr - cr * sp * sy) + cpm * sym * (sr * sy + cr * cy * sp)) / std::sqrt(std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2)) -
                ((2 * ((sr * sy + cr * cy * sp) * (cym * srm - crm * spm * sym) - (cy * sr - cr * sp * sy) * (srm * sym + crm * cym * spm)) * ((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm) +
                    2 * ((sr * sy + cr * cy * sp) * (crm * cym + spm * srm * sym) - (cy * sr - cr * sp * sy) * (crm * sym - cym * spm * srm)) * ((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm)) * (cpm * sym * (cy * sr - cr * sp * sy) - cpm * cym * (sr * sy + cr * cy * sp) + cp * cr * spm)) /
                    (2 * std::pow(std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2), 1.5)))) /
            (std::pow((sr * sy + cr * cy * sp) * (srm * sym + crm * cym * spm) + (cy * sr - cr * sp * sy) * (cym * srm - crm * spm * sym) + cp * cpm * cr * crm, 2) + std::pow((sr * sy + cr * cy * sp) * (crm * sym - cym * spm * srm) + (cy * sr - cr * sp * sy) * (crm * cym + spm * srm * sym) - cp * cpm * cr * srm, 2) + std::pow(cpm * sym * (cy * sr - cr * sp * sy) - cpm * cym * (sr * sy + cr * cy * sp) + cp * cr * spm, 2));
    double H50m = 0,
        H51m = 0,
        H52m = 0,
        H53m = 0,
        H54m = (((cp * cy * cym * spm - cpm * sp + cp * spm * sy * sym) / (cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr) + ((sp * spm + cp * cpm * cy * cym + cp * cpm * sy * sym) * (spm * sym * (cr * cy + sp * sr * sy) - cym * spm * (cr * sy - cy * sp * sr) + cp * cpm * sr)) / std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)) *
        std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)) / (std::pow(sp * spm + cp * cpm * cy * cym + cp * cpm * sy * sym, 2) + std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)), H55m =
        (((cp * cpm * cy * sym - cp * cpm * cym * sy) / (cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr) - ((cpm * cym * (cr * cy + sp * sr * sy) + cpm * sym * (cr * sy - cy * sp * sr)) * (sp * spm + cp * cpm * cy * cym + cp * cpm * sy * sym)) / std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)) *
            std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2)) / (std::pow(sp * spm + cp * cpm * cy * cym + cp * cpm * sy * sym, 2) + std::pow(cpm * cym * (cr * sy - cy * sp * sr) - cpm * sym * (cr * cy + sp * sr * sy) + cp * spm * sr, 2));

    // jacobian wrt the marker state (x, y, z, r, p, y)
    // the way the seen landmark affects its own position.
    Jacobian << H00m, H01m, H02m, H03m, H04m, H05m, H10m, H11m, H12m, H13m, H14m, H15m, H20m, H21m, H22m, H23m, H24m, H25m, H30m, H31m, H32m, H33m, H34m, H35m, H40m, H41m, H42m, H43m, H44m, H45m, H50m, H51m, H52m, H53m, H54m, H55m;

    return Jacobian;
}

const Eigen::VectorXd &
Marker::getInverseObservationModel(const DroneState &drone_state, const Eigen::VectorXd &observation)
{
    double roll = drone_state.euler(0), pitch = drone_state.euler(1), yaw = drone_state.euler(2);

    Eigen::Matrix4d Tr;
    get_homogeneous_transformation(drone_state.position(0), drone_state.position(1), drone_state.position(2), roll, pitch, yaw, Tr);

    Eigen::Matrix4d Tc;
    get_homogeneous_transformation(0.0, 0.0, 0.0, -M_PI, 0.0, -M_PI_2, Tc);

    Eigen::Matrix4d marker_local_pos;
    get_homogeneous_transformation(observation(0), observation(1), observation(2), observation(3), observation(4), observation(5), marker_local_pos);

    // get markers position in the global reference frame (inverse observation model)
    Eigen::Matrix4d marker_global_pos = Tr * Tc * marker_local_pos;

    Eigen::Vector3d euler_m;
    get_euler_from_matrix(marker_global_pos, euler_m);

    m_state << marker_global_pos(0, 3), marker_global_pos(1, 3), marker_global_pos(2, 3), euler_m(0), euler_m(1), euler_m(2);

    return m_state;
}

Eigen::MatrixXd
Marker::getInverseJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const
{
    // marker
    double xm = m_state(0), ym = m_state(1), zm = m_state(2);
    double pitch_m = m_state(4), yaw_m = m_state(5);
    double cpm = std::cos(pitch_m), cym = std::cos(yaw_m);
    double spm = std::sin(pitch_m), sym = std::sin(yaw_m);
    // robot
    double cr = std::cos(drone_state.euler(0)), cp = std::cos(drone_state.euler(1)), cy = std::cos(drone_state.euler(2));
    double sr = std::sin(drone_state.euler(0)), sp = std::sin(drone_state.euler(1)), sy = std::sin(drone_state.euler(2));

    // jacobian of the inverse observation model wrt the state variables.
    Eigen::MatrixXd J_inv_h_x = Eigen::MatrixXd::Identity(state_size, drone_state_size);

    J_inv_h_x(0, 3) = xm * (cr * cy + sp * sr * sy) - zm * (cy * sr - cr * sp * sy) + ym * cp * sy;
    J_inv_h_x(1, 3) = xm * (cr * sy - cy * sp * sr) - zm * (sr * sy + cr * cy * sp) - ym * cp * cy;
    J_inv_h_x(3, 3) = 0;
    J_inv_h_x(5, 3) = ((std::pow(spm * (cy * sr - cr * sp * sy) + cpm * cym * (cr * cy + sp * sr * sy) + cp * cpm * sy * sym, 2) / std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2) + 1) * std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2)) /
        (std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2) + std::pow(spm * (cy * sr - cr * sp * sy) + cpm * cym * (cr * cy + sp * sr * sy) + cp * cpm * sy * sym, 2));

    return J_inv_h_x;
}

Eigen::MatrixXd
Marker::getInverseJacobianWrtLandmarkState(const DroneState &drone_state) const
{
    // marker
    double roll_m = m_state(3), pitch_m = m_state(4), yaw_m = m_state(5);
    double crm = std::cos(roll_m), cpm = std::cos(pitch_m), cym = std::cos(yaw_m);
    double srm = std::sin(roll_m), spm = std::sin(pitch_m), sym = std::sin(yaw_m);
    // robot
    double cr = std::cos(drone_state.euler(0)), cp = std::cos(drone_state.euler(1)), cy = std::cos(drone_state.euler(2));
    double sr = std::sin(drone_state.euler(0)), sp = std::sin(drone_state.euler(1)), sy = std::sin(drone_state.euler(2));

    // Jacobian of the inverse observation model wrt the observation variables.
    Eigen::MatrixXd J_inv_h_z = Eigen::MatrixXd::Zero(state_size, state_size);

    double J00 = cr * sy - cy * sp * sr,
        J01 = -cp * cy,
        J02 = -sr * sy - cr * cy * sp,
        J03 = 0,
        J04 = 0,
        J05 = 0;
    double J10 = -cr * cy - sp * sr * sy,
        J11 = -cp * sy,
        J12 = cy * sr - cr * sp * sy,
        J13 = 0,
        J14 = 0,
        J15 = 0;
    double J20 = -cp * sr,
        J21 = sp,
        J22 = -cp * cr,
        J23 = 0,
        J24 = 0,
        J25 = 0;
    double J30 = 0,
        J31 = 0,
        J32 = 0,
        J33 = ((std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2) / std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + 1) * std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2)) /
        (std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2)),
        J34 = -(((cp * cr * spm * srm + cpm * sp * srm * sym - cp * cpm * cym * sr * srm) / (sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm) + ((sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm) * (cp * cr * crm * spm + cpm * crm * sp * sym - cp * cpm * crm * cym * sr)) / std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2)) *
        std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2)) / (std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2)),
        J35 = (((sp * (crm * sym - cym * spm * srm) - cp * sr * (crm * cym + spm * srm * sym)) / (sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm) - ((sp * (srm * sym + crm * cym * spm) - cp * sr * (cym * srm - crm * spm * sym)) * (sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm)) / std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2)) *
        std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2)) / (std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2));
    double J40 = 0,
        J41 = 0,
        J42 = 0,
        J43 = 0,
        J44 = -((std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2)) *
        ((cp * cpm * cr - sp * spm * sym + cp * cym * spm * sr) / std::sqrt(std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2)) +
            ((2 * (sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm) * (cp * cr * crm * spm + cpm * crm * sp * sym - cp * cpm * crm * cym * sr) - 2 * (sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm) * (cp * cr * spm * srm + cpm * sp * srm * sym - cp * cpm * cym * sr * srm)) * (cpm * sp * sym + cp * cr * spm - cp * cpm * cym * sr)) /
                (2 * std::pow(std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2), 1.5)))) /
        (std::pow(cpm * sp * sym + cp * cr * spm - cp * cpm * cym * sr, 2) + std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2)),
        J45 = -(((cpm * cym * sp + cp * cpm * sr * sym) / std::sqrt(std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2)) +
        ((2 * (sp * (srm * sym + crm * cym * spm) - cp * sr * (cym * srm - crm * spm * sym)) * (sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm) + 2 * (sp * (crm * sym - cym * spm * srm) - cp * sr * (crm * cym + spm * srm * sym)) * (sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm)) * (cpm * sp * sym + cp * cr * spm - cp * cpm * cym * sr)) /
            (2 * std::pow(std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2), 1.5))) * (std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2))) /
        (std::pow(cpm * sp * sym + cp * cr * spm - cp * cpm * cym * sr, 2) + std::pow(sp * (cym * srm - crm * spm * sym) + cp * sr * (srm * sym + crm * cym * spm) + cp * cpm * cr * crm, 2) + std::pow(sp * (crm * cym + spm * srm * sym) + cp * sr * (crm * sym - cym * spm * srm) - cp * cpm * cr * srm, 2));
    double J50 = 0,
        J51 = 0,
        J52 = 0,
        J53 = 0,
        J54 = (((cym * spm * (cr * cy + sp * sr * sy) - cpm * (cy * sr - cr * sp * sy) + cp * spm * sy * sym) / (spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym) +
        ((spm * (cy * sr - cr * sp * sy) + cpm * cym * (cr * cy + sp * sr * sy) + cp * cpm * sy * sym) * (cpm * (sr * sy + cr * cy * sp) - cym * spm * (cr * sy - cy * sp * sr) + cp * cy * spm * sym)) / std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2)) * std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2)) /
        (std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2) + std::pow(spm * (cy * sr - cr * sp * sy) + cpm * cym * (cr * cy + sp * sr * sy) + cp * cpm * sy * sym, 2)),
        J55 = (((cpm * sym * (cr * cy + sp * sr * sy) - cp * cpm * cym * sy) / (spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym) - ((cpm * sym * (cr * sy - cy * sp * sr) + cp * cpm * cy * cym) * (spm * (cy * sr - cr * sp * sy) + cpm * cym * (cr * cy + sp * sr * sy) + cp * cpm * sy * sym)) / std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2)) *
        std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2)) / (std::pow(spm * (sr * sy + cr * cy * sp) + cpm * cym * (cr * sy - cy * sp * sr) - cp * cpm * cy * sym, 2) + std::pow(spm * (cy * sr - cr * sp * sy) + cpm * cym * (cr * cy + sp * sr * sy) + cp * cpm * sy * sym, 2));

    J_inv_h_z << J00, J01, J02, J03, J04, J05,
        J10, J11, J12, J13, J14, J15,
        J20, J21, J22, J23, J24, J25,
        J30, J31, J32, J33, J34, J35,
        J40, J41, J42, J43, J44, J45,
        J50, J51, J52, J53, J54, J55;

    return J_inv_h_z;
}
