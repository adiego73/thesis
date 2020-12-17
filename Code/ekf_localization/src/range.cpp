#include "ekf_localization/range.hpp"
Eigen::VectorXd
Range::getObservationModel(const DroneState &drone_state) const
{
    Eigen::VectorXd model(1);
    model << drone_state.position(2) + LASER_RANGE_BIAS;

    return model;
}

Eigen::MatrixXd
Range::getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const
{
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(state_size, drone_state_size);

    // this only affects the drone's altitude.
    Jacobian(0,2) = 1;
    
    return Jacobian;
}

Range::Range() :
    Landmark(-1, LandmarkType::RANGE, 1) {}
