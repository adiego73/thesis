#include "ekf_localization/commons.hpp"

void
get_homogeneous_transformation(double x, double y, double z, double roll, double pitch, double yaw, Eigen::Matrix4d &out)
{
    // from odom reference frame to inertial/map reference frame
    double T11 = std::cos(yaw) * std::cos(pitch);
    double T12 = std::cos(yaw) * std::sin(pitch) * std::sin(roll) - std::sin(yaw) * std::cos(roll);
    double T13 = std::cos(yaw) * std::sin(pitch) * std::cos(roll) + std::sin(yaw) * std::sin(roll);

    double T21 = std::sin(yaw) * std::cos(pitch);
    double T22 = std::sin(yaw) * std::sin(pitch) * std::sin(roll) + std::cos(yaw) * std::cos(roll);
    double T23 = std::sin(yaw) * std::sin(pitch) * std::cos(roll) - std::cos(yaw) * std::sin(roll);

    double T31 = -std::sin(pitch);
    double T32 = std::cos(pitch) * std::sin(roll);
    double T33 = std::cos(pitch) * std::cos(roll);

    out << T11, T12, T13, x, T21, T22, T23, y, T31, T32, T33, z, 0, 0, 0, 1;
}

void
get_euler_from_quaternion(tf2::Quaternion q, Eigen::Vector3d &out)
{
    out(0) = 0;
    out(1) = 0;
    out(2) = 0;

    double norm = std::sqrt(std::pow(q.x(), 2) + std::pow(q.y(), 2) + std::pow(q.z(), 2) + std::pow(q.w(), 2));
    if (norm != 0)
    {
        q.setX(q.x() / norm);
        q.setY(q.y() / norm);
        q.setZ(q.z() / norm);
        q.setW(q.w() / norm);

        double A11 = std::pow(q.x(), 2) - std::pow(q.y(), 2) - std::pow(q.z(), 2) + std::pow(q.w(), 2);
        double A12 = 2 * q.x() * q.y() + 2 * q.z() * q.w();
        double A13 = 2 * q.x() * q.z() - 2 * q.y() * q.w();
        double A23 = 2 * q.x() * q.w() + 2 * q.y() * q.z();
        double A33 = -std::pow(q.x(), 2) - std::pow(q.y(), 2) + std::pow(q.z(), 2) + std::pow(q.w(), 2);

        out(0) = wrapToPi(std::atan2(A23, A33));
        out(1) = wrapToPi(-std::asin(A13));
        out(2) = wrapToPi(std::atan2(A12, A11));
    }
}

void
get_euler_from_matrix(const Eigen::Matrix4d &R, Eigen::Vector3d &out)
{
    out(0) = wrapToPi(std::atan2(R(2, 1), R(2, 2)));
    out(1) = wrapToPi(std::atan2(-R(2, 0), std::sqrt(std::pow(R(2, 1), 2) + std::pow(R(2, 2), 2))));
    out(2) = wrapToPi(std::atan2(R(1, 0), R(0, 0)));
}

/**
 * https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
 * @param R
 * @param out
 */
void
get_quaternion_from_matrix(const Eigen::Matrix3d &R, tf2::Quaternion &out)
{
    double x, y, z, w, s;

    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    if (trace > 0)
    {
        s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (R(2, 1) - R(1, 2)) * s;
        y = (R(0, 2) - R(2, 0)) * s;
        z = (R(1, 0) - R(0, 1)) * s;
    }
    else
    {
        if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
        {
            s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            w = (R(2, 1) - R(1, 2)) / s;
            x = 0.25 * s;
            y = (R(0, 1) + R(1, 0)) / s;
            z = (R(0, 2) + R(2, 0)) / s;
        }
        else if (R(1, 1) > R(2, 2))
        {
            s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
            w = (R(0, 2) - R(2, 0)) / s;
            x = (R(0, 1) + R(1, 0)) / s;
            y = 0.25 * s;
            z = (R(1, 2) + R(2, 1)) / s;
        }
        else
        {
            s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
            w = (R(1, 0) - R(0, 1)) / s;
            x = (R(0, 2) + R(2, 0)) / s;
            y = (R(1, 2) + R(2, 1)) / s;
            z = 0.25 * s;
        }
    }

    out.setX(x);
    out.setY(y);
    out.setZ(z);
    out.setW(w);
}

void
wrapTo2PiInPlace(double &angle)
{
    bool was_neg = angle < 0;
    angle = std::fmod(angle, 2.0 * M_PI);
    if (was_neg)
        angle += 2.0 * M_PI;
}

void
wrapToPiInPlace(double &angle)
{
    angle += M_PI;
    wrapTo2PiInPlace(angle);
    angle -= M_PI;
}

double
wrapToPi(double angle)
{
    wrapToPiInPlace(angle);
    return angle;
}

double
radToDeg(double angle)
{
    return angle * 180 / M_PI;
}

std::string
get_landmark_name(const LandmarkType& type) {
    switch (type) {
        case LandmarkType::POLE:
            return "POLE";
        case LandmarkType::MARKER:
            return "MARKER";
        case LandmarkType::RANGE:
            return "LASER_RANGE";
        default:
            return "UNKNOWN";
    }
}