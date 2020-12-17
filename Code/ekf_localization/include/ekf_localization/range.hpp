#ifndef EKF_LOCALIZATION_RANGE_HPP
#define EKF_LOCALIZATION_RANGE_HPP

#include "landmark.hpp"
#include "map_manager.hpp"
class Range : public Landmark
{
private:
    static constexpr double LASER_RANGE_BIAS = -0.2;

public:
    explicit Range();

    Eigen::VectorXd
    getObservationModel(const DroneState &drone_state) const override;

    Eigen::MatrixXd
    getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const override;
};

#endif//EKF_LOCALIZATION_RANGE_HPP
