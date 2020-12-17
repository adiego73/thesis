#ifndef EKF_LOCALIZATION_EKF_HPP
#define EKF_LOCALIZATION_EKF_HPP

#include "commons.hpp"
#include "landmark.hpp"

/**
 * EKF Base class.
 *
 * \hat{\mu}    = g( u_t , \mu_{t}  )
 * \hat\Sigma   = G_t \Sigma_{t-1} G_t^T + R_t
 * K_t          = \hat\Sigma H_t^T ( H_t \hat\Sigma H_t^T + Q_t )^{-1}
 * \mu_{t+1}    = \hat\mu + K_t ( z_t - h ( \hat\mu ) )
 * \Sigma_{t+1} = ( I - K_t H_t ) \hat\Sigma
 */
class EKF
{
protected:
    // state vector
    Eigen::VectorXd m_state;
    // covariance matrix
    Eigen::MatrixXd m_covariance;

    /**
     * Motion model
     * @param control (Eigen::VectorXf) vector
     * @return Eigen::VectorXf
     */
    virtual void
    g(const Eigen::VectorXd &control) = 0;

    /**
     * Observation model
     * @return Eigen::VectorXf
     */
    virtual Eigen::VectorXd
    h(const Landmark &landmark) const = 0;

    /**
     * Jacobian of the observation model
     * @return
     */
    virtual Eigen::MatrixXd
    H(const Landmark &landmark) const = 0;

    /**
     * Jacobian of the motion model
     * @return
     */
    virtual Eigen::MatrixXd
    G(const Eigen::VectorXd &control) const = 0;

    /**
     * chi2 upper limit to validate the observation confidence.
     * @param LandmarkType
     * @return
     */
    virtual double
    chiSquareLimit(const LandmarkType &type) const = 0;

public:
    explicit EKF(int state_elements) :
        m_state(Eigen::VectorXd::Zero(state_elements)),
        m_covariance(1e-9 * Eigen::MatrixXd::Identity(state_elements, state_elements))
    {
    }

    virtual ~EKF() = default;

    virtual Eigen::MatrixXd
    getControlCovariance() const = 0;

    virtual Eigen::MatrixXd
    getObservationCovariance(const LandmarkType &type) const = 0;

    /**
     * Do the prediction step in the EKF algorithm. As argument it uses the control vector, and it updates the
     * state vector and the covariance matrix
     * @param control
     */
    void
    prediction(const Eigen::VectorXd &control)
    {
        // prediction
        this->g(control);
        Eigen::MatrixXd G = this->G(control);
        this->m_covariance = G * this->m_covariance * G.transpose() + this->getControlCovariance();
    }

    /**
     * Do the correction step in the EKF algorithm. As argument it uses the observation vector, and the landmark id.
     * Finally, it updates the state vector and the covariance matrix after seeing a landmark.
     * @param observations
     */
    void
    correction(const Eigen::VectorXd &observations, const Landmark &landmark)
    {
        // correction
        Eigen::MatrixXd H = this->H(landmark);
        Eigen::MatrixXd S = H * this->m_covariance * H.transpose() + this->getObservationCovariance(landmark.TYPE);
        Eigen::MatrixXd S_inverse = S.inverse();
        Eigen::MatrixXd K = this->m_covariance * H.transpose() * S_inverse;

        Eigen::VectorXd innovation = this->innovation(observations, landmark);

        // if the error square is less than the precomputed chi2 upper limit, we perform the update.
        // If e2 is not greater, it means that the observation is not very reliable.
        // For more info, https://www.coursera.org/lecture/battery-state-of-charge/3-3-5-can-we-automatically-detect-bad-measurements-with-a-kalman-filter-tc7ce
        double e2 = innovation.transpose() * S_inverse * innovation;
        if (e2 < this->chiSquareLimit(landmark.TYPE))
        {
            this->m_state = this->m_state + K * innovation;
            this->m_covariance = (Eigen::MatrixXd::Identity(this->m_covariance.rows(), this->m_covariance.cols()) - (K * H)) * this->m_covariance;
        }
        else
        {
            std::stringstream ss;
            ss << innovation.transpose();
            ROS_INFO("chi2 (%f) less than e2 (%f) for landmark (%s) id %d :: Innovation %s", this->chiSquareLimit(landmark.TYPE), e2, get_landmark_name(landmark.TYPE).c_str(), landmark.ID, ss.str().c_str());
        }
    }

    virtual Eigen::VectorXd
    innovation(const Eigen::VectorXd &observations, const Landmark &landmark) const
    {
        Eigen::VectorXd observation_hat = h(landmark);
        return observations - observation_hat;
    }
};

#endif//EKF_LOCALIZATION_EKF_HPP
