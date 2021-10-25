#ifndef __POS_CONTROL_H__
#define __POS_CONTROL_H__

#include <iostream>
#include <Eigen/Geometry>
#include <offboard_so3/common.hpp>

class PosCtrl
{
public:
    PosCtrl() {}
    ~PosCtrl() {}
    void init(const int &mode,
              const double &rate,
              const Eigen::Vector4d &kp,
              const Eigen::Vector4d &ki,
              const Eigen::Vector4d &kd,
              const double &xysatur,
              const double &zsatur,
              const double &yawratesatur,
              const double &xy_dsatur,
              const double &xy_i_errsatur,
              const double &yaw_dsatur,
              const double &yawrate_i_errsatur);
    void reset(void);

    void setPosCtrlMode(const int &mode);
    void setPosCtrlRate(const double &rate);
    void setPosCtrlParam(const Eigen::Vector4d &kp,
                         const Eigen::Vector4d &ki,
                         const Eigen::Vector4d &kd);
    void setPositionState(const Eigen::Vector3d &position, const double &yaw);
    void updateVelocityCmd(const Eigen::Vector3d &des_pos, const double &des_yaw, 
                           const Eigen::Vector3d &vel_ff);
    
    const Eigen::Vector3d &getVelocityCmdENU(void);
    const Eigen::Vector3d &getVelocityCmdFLU(void);
    const double getYawCmd(void);
    const double getYawrateCmd(void);

    double XY_Satur;
    double Z_Satur;
    double YAWRATE_Satur;
    double XY_D_Satur;
    double XY_I_Err_Satur;
    double YAWRATE_D_Satur;
    double YAWRATE_I_Err_Satur;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    int mode_;
    double rate_;
    
    Eigen::Vector4d kp_;
    Eigen::Vector4d ki_;
    Eigen::Vector4d kd_;

    Eigen::Vector3d pos_;
    double yaw_;

    Eigen::Vector3d vel_cmd_;
    Eigen::Vector3d vel_cmd_body_;
    double yawrate_cmd_;
    double yaw_cmd_;

    Eigen::Vector3d err_pos_last_;
    Eigen::Vector3d err_pos_integrate_;
    double err_yaw_last_;
    double err_yaw_integrate_;
};

#endif