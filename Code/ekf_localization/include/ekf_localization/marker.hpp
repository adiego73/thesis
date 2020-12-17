#ifndef EKF_LOCALIZATION_MARKER_HPP
#define EKF_LOCALIZATION_MARKER_HPP

#include "commons.hpp"
#include "landmark.hpp"

class Marker : public Landmark
{

public:
    explicit Marker(int id);

    Marker(int id, double x, double y, double z, double roll, double pitch, double yaw);

    Eigen::VectorXd
    getObservationModel(const DroneState &drone_state) const override;

    Eigen::MatrixXd
    getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const override;

    Eigen::MatrixXd
    getJacobianWrtLandmarkState(const DroneState &drone_state) const override;

    const Eigen::VectorXd &
    getInverseObservationModel(const DroneState &drone_state, const Eigen::VectorXd &observation) override;

    Eigen::MatrixXd
    getInverseJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const override;

    Eigen::MatrixXd
    getInverseJacobianWrtLandmarkState(const DroneState &drone_state) const override;
};

#endif//EKF_LOCALIZATION_MARKER_HPP
