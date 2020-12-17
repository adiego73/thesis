#ifndef EKF_LOCALIZATION_LANDMARK_HPP
#define EKF_LOCALIZATION_LANDMARK_HPP

#include "commons.hpp"

class Landmark
{
protected:
    /**
     * State vector of the landmark. Depending on the landmark, it can be x, y, z or x, y, z, roll, pitch, yaw.
     */
    Eigen::VectorXd m_state;

    /*
     * State vector size.
     */
    const int state_size;

public:
    /**
     * Landmark id
     */
    const int ID;

    /**
     * Landmark type.
     */
    const LandmarkType TYPE;

    Landmark(int id, LandmarkType type, int landmark_state_size) :
        state_size(landmark_state_size), ID(id), m_state(Eigen::VectorXd::Zero(landmark_state_size)), TYPE(type){};

    virtual ~Landmark() = default;

    /**
     * Computes and returns the observation model vector, depending on the landmark type it can be the range an bearing or the pose.
     * @param drone_state
     * @return
     */
    virtual Eigen::VectorXd
    getObservationModel(const DroneState &drone_state) const = 0;

    /**
     * Computes and returns the Jacobian of the observation model with respect to the drone state.
     * @param drone_state
     * @param drone_state_size
     * @return
     */
    virtual Eigen::MatrixXd
    getJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const = 0;

    /**
     * Computes and returns the Jacobian of the observation model with respect to the landmark state.
     * @param drone_state
     * @return
     */
    virtual Eigen::MatrixXd
    getJacobianWrtLandmarkState(const DroneState &drone_state) const
    {
        throw std::runtime_error("Method getJacobianWrtLandmarkState is not implemented");
    }

    /**
     * Computes and returns the inverse observation model vector based on the drone state and the observation.
     * Also, it updates the landmark's state vector.
     * @param drone_state
     * @param observation
     * @return
     */
    virtual const Eigen::VectorXd &
    getInverseObservationModel(const DroneState &drone_state, const Eigen::VectorXd &observation)
    {
        throw std::runtime_error("Method getInverseObservationModel is not implemented");
    }

    /**
     * Computes and returns the Jacobian of the inverse observation model with respect to the drone state.
     * @param drone_state
     * @param drone_state_size
     * @return
     */
    virtual Eigen::MatrixXd
    getInverseJacobianWrtDroneState(const DroneState &drone_state, int drone_state_size) const
    {
        throw std::runtime_error("Method getInverseJacobianWrtDroneState is not implemented");
    }

    /**
     * Computes and returns the Jacobian of the inverse observation model with respect to the landmark state.
     * @param drone_state
     * @return
     */
    virtual Eigen::MatrixXd
    getInverseJacobianWrtLandmarkState(const DroneState &drone_state) const
    {
        throw std::runtime_error("Method getInverseJacobianWrtLandmarkState is not implemented");
    }

    /**
     * Updates the landmark's state vector
     * @param state
     */
    void
    setState(const Eigen::VectorXd &state)
    {
        m_state = state;
    }

    /**
     * Get value of state at position 'index'
     * @param index
     * @return
     */
    double
    getState(int index) const
    {
        return m_state(index);
    }

    /**
     * Returns the landmark's state vector.
     * @return
     */
    const Eigen::VectorXd &
    getState() const
    {
        return m_state;
    }

    constexpr int
    getStateSize() const
    {
        return state_size;
    }
};

#endif//EKF_LOCALIZATION_LANDMARK_HPP
