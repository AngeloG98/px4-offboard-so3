#ifndef __COMMON_H__
#define __COMMON_H__

#include <Eigen/Geometry>

const double deg2rad = 3.1415926535798/180.0;
const double rad2deg = 180.0/3.1415926535798;

template <typename T>
void limit(T& a, T max)
{
    if(a > max)
        a = max;
    if(a < -max)
        a = -max;
}

template <typename T>
void limit(T& a, T& b, T max)
{
    double c = sqrt(a * a + b * b);
    if (c>max)
    {
        a = a / c * max;
        b = b / c * max;
    }
}

template <typename T>
void limit(T& a, T& b, T& c, T max)
{
    double d = sqrt(a * a + b * b + c * c);
    if (d>max)
    {
        a = a / d * max;
        b = b / d * max;
        c = c / d * max;
    }
}

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) 
{
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) 
{
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0) {
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}

Eigen::Vector3d quat2EulerAngle(Eigen::Vector4d quat)
{
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];

    double roll, pitch, yaw;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    Eigen::Vector3d euler;
    euler = Eigen::Vector3d(roll, pitch, yaw);

    return euler;
}

#endif