//
// Created by diego on 9/3/20.
//

#ifndef EKF_LOCALIZATION_POLE_HPP
#define EKF_LOCALIZATION_POLE_HPP

#include "commons.hpp"
#include "landmark.hpp"

class Pole : public Landmark
{
public:
    Pole(int id, double x, double y, double z) :
        Landmark(id, LandmarkType::POLE, 3)
    {
        m_state(0) = x;
        m_state(1) = y;
        m_state(2) = z;
    }

    Eigen::VectorXd
    getObservationModel(const DroneState &drone_state) const override;

    Eigen::MatrixXd
    getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const override;

    // It does not make sense to implement the Inverse observation model and it's Jacobian computations.
};

#endif//EKF_LOCALIZATION_POLE_HPP
