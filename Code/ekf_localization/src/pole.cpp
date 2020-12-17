#include "ekf_localization/pole.hpp"

Eigen::VectorXd
Pole::getObservationModel(const DroneState &drone_state) const
{
    Eigen::Vector3d h_hat(state_size);

    double x = drone_state.position(0),
        y = drone_state.position(1),
        z = drone_state.position(2);

    Eigen::Matrix4d T;
    get_homogeneous_transformation(x, y, z, drone_state.euler(0), drone_state.euler(1), drone_state.euler(2), T);

    Eigen::Vector4d pole_pos;
    pole_pos << m_state, 1;

    // transforms the poles position from the map reference frame to the drone's reference frame
    Eigen::Vector4d pole_pos_body_frame = T.inverse() * pole_pos;
    double xp = pole_pos_body_frame(0), yp = pole_pos_body_frame(1), zp = pole_pos_body_frame(2);

    // distance
    h_hat(0) = std::sqrt(std::pow(xp, 2) + std::pow(yp, 2));
    // azimuth
    h_hat(1) = wrapToPi(std::atan2(yp, xp));
    // elevation
    h_hat(2) = wrapToPi(std::atan2(zp, h_hat(0)));

    return h_hat;
}

Eigen::MatrixXd
Pole::getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const
{
    double x = drone_state.position(0), y = drone_state.position(1), z = drone_state.position(2);

    double cr = std::cos(drone_state.euler(0)), cp = std::cos(drone_state.euler(1)), cy = std::cos(drone_state.euler(2));
    double sr = std::sin(drone_state.euler(0)), sp = std::sin(drone_state.euler(1)), sy = std::sin(drone_state.euler(2));
    double xp = m_state(0), yp = m_state(1), zp = m_state(2);

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
    double crcy_spsrsy = crcy + spsrsy;
    double srsy_crcysp = srsy + crcysp;
    double cysr_crspsy = cysr - crspsy;

    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(state_size, drone_state_size);

    // distance
    double jacobian_distance_div = 2 * std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 0.5);
    // d_h/d_x
    double H00 = -(2 * (crsy_cyspsr) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy) + 2 * cpcy * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) / jacobian_distance_div;
    // d_h/d_y
    double H01 = (2 * (crcy_spsrsy) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy) - 2 * cpsy * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) / jacobian_distance_div;
    // d_h/d_z
    double H02 = (2 * sp * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy) + 2 * cpsr * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy)) / jacobian_distance_div;
    // d_h/d_yaw
    double H03 = (2 * (xp * (crcy_spsrsy) + yp * (crsy_cyspsr) - x * crcy - y * crsy + y * cyspsr - x * spsrsy) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy) - 2 * (y * cpcy - yp * cpcy - x * cpsy + xp * cpsy) * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) / jacobian_distance_div;

    // azimuth
    double jacobian_azimuth_div = (std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2));
    // d_h/d_x
    double H10 = (((crsy_cyspsr) / (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy) - (cpcy * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy)) / std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) * std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) / jacobian_azimuth_div;
    // d_h/d_y
    double H11 = -(((crcy_spsrsy) / (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy) + (cpsy * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy)) / std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) * std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) / jacobian_azimuth_div;
    // d_h/d_z
    double H12 = (((sp * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy)) / std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) - (cpsr) / (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) * std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) / jacobian_azimuth_div;
    // d_h/d_yaw
    double
        H13 = -(((xp * (crcy_spsrsy) + yp * (crsy_cyspsr) - x * crcy - y * crsy + y * cyspsr - x * spsrsy) / (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy) + ((y * cpcy - yp * cpcy - x * cpsy + xp * cpsy) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy)) / std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) * std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2)) /
        jacobian_azimuth_div;

    // elevation
    double jacobian_elevation_div = std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(yp * (cysr_crspsy) - xp * (srsy_crcysp) + z * cpcr - zp * cpcr - y * cysr + x * srsy + x * crcysp + y * crspsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2);
    // d_h/d_x
    double H20 = -((std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2)) * ((srsy_crcysp) / std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 0.5) +
        ((2 * (crsy_cyspsr) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy) + 2 * cpcy * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) * (yp * (cysr_crspsy) - xp * (srsy_crcysp) + z * cpcr - zp * cpcr - y * cysr + x * srsy + x * crcysp + y * crspsy)) /
            (2 * std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 1.5)))) / jacobian_elevation_div;
    // d_h/d_y
    double H21 = ((std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2)) * ((cysr_crspsy) / std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 0.5) +
        ((2 * (crcy_spsrsy) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy) - 2 * cpsy * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) * (yp * (cysr_crspsy) - xp * (srsy_crcysp) + z * cpcr - zp * cpcr - y * cysr + x * srsy + x * crcysp + y * crspsy)) /
            (2 * std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 1.5)))) / jacobian_elevation_div;
    // d_h/d_z
    double H22 = -(((cpcr) / std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 0.5) -
        ((2 * sp * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy) + 2 * cpsr * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy)) * (yp * (cysr_crspsy) - xp * (srsy_crcysp) + z * cpcr - zp * cpcr - y * cysr + x * srsy + x * crcysp + y * crspsy)) /
            (2 * std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 1.5))) * (std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2))) / jacobian_elevation_div;
    // d_h/d_yaw
    double H23 = (((xp * (cysr_crspsy) + yp * (srsy_crcysp) - x * cysr - y * srsy - y * crcysp + x * crspsy) / std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 0.5) +
        ((2 * (xp * (crcy_spsrsy) + yp * (crsy_cyspsr) - x * crcy - y * crsy + y * cyspsr - x * spsrsy) * (xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy) - 2 * (y * cpcy - yp * cpcy - x * cpsy + xp * cpsy) * (z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy)) * (yp * (cysr_crspsy) - xp * (srsy_crcysp) + z * cpcr - zp * cpcr - y * cysr + x * srsy + x * crcysp + y * crspsy)) /
            (2 * std::pow(std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2), 1.5))) * (std::pow(z * sp - zp * sp - x * cpcy + xp * cpcy - y * cpsy + yp * cpsy, 2) + std::pow(xp * (crsy_cyspsr) - yp * (crcy_spsrsy) + y * crcy + z * cpsr - zp * cpsr - x * crsy + x * cyspsr + y * spsrsy, 2))) / jacobian_elevation_div;

    // it affects only the position of the robot, not the landmarks
    Jacobian << H00, H01, H02, H03, H10, H11, H12, H13, H20, H21, H22, H23;

    return Jacobian;
}