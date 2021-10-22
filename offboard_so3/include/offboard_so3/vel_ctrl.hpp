#ifndef __VEL_CONTROL_H__
#define __VEL_CONTROL_H__

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <offboard_so3/common.hpp>

class VelCtrl
{
public:
    VelCtrl();
    VelCtrl(const int &mode, const double &rate);
    void reset(void);

    void setVelCtrlMode(const int &mode);
    void setVelCtrlRate(const double &rate);
    void setModelParam(const double &mass, const double &fmax);
    void setVelCtrlParam(const Eigen::Vector4d &Kp,
                         const Eigen::Vector4d &Ki,
                         const Eigen::Vector4d &Kd);
    void setVelocityState(const Eigen::Vector3d &velocity, const double &yaw);
    void updateAttitudeCmd(const Eigen::Vector3d &des_vel, const double &des_yaw,
                           const Eigen::Vector3d &acc_ff);

    const Eigen::Matrix3d &getAttitudeCmdRotMat(void);
    const Eigen::Vector4d &getAttitudeCmdQuat(void);
    const Eigen::Vector3d &getAttitudeCmdEuler(void);
    const double &getYawrateCmd(void);
    const double &getThrottleCmd(void);
    
    Eigen::Vector3d Acc_Satur;
    double Pitch_Satur;
    double Roll_Satur;
    double YAWRATE_Satur;
    Eigen::Vector3d XYZ_D_Satur;
    Eigen::Vector3d XYZ_I_Err_Satur;
    double YAWRATE_D_Satur;
    double YAWRATE_I_Err_Satur;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

    int mode_;
    double rate_;
    
    double mass_;
    double fmax_;
    Eigen::Vector3d g_;

    Eigen::Vector4d kp_;
    Eigen::Vector4d ki_;
    Eigen::Vector4d kd_;

    Eigen::Vector3d vel_;
    double yaw_;

    Eigen::Vector3d acc_cmd_;
    Eigen::Matrix3d RotMat_cmd_;
    Eigen::Vector4d Quat_cmd_;
    Eigen::Vector3d Euler_cmd_;
    double yaw_cmd_;
    double yawrate_cmd_;
    double thr_cmd_;

    Eigen::Vector3d err_vel_last_;
    Eigen::Vector3d err_vel_integrate_;
    double err_yaw_last_;
    double err_yaw_integrate_;
};

#endif