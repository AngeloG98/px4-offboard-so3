#include <offboard_so3/pos_ctrl.hpp>

void PosCtrl::init(const int &mode,
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
                   const double &yawrate_i_errsatur)
{
    mode_ = mode;
    rate_ = rate;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    XY_Satur = xysatur;
    Z_Satur = zsatur;
    YAWRATE_Satur = yawratesatur;
    XY_D_Satur = xy_dsatur;
    XY_I_Err_Satur = xy_i_errsatur;
    YAWRATE_D_Satur = yaw_dsatur;
    YAWRATE_I_Err_Satur = yawrate_i_errsatur;

    vel_cmd_ << 0.0, 0.0, 0.0;
    vel_cmd_body_ << 0.0, 0.0, 0.0;
    yawrate_cmd_ = 0;
    yaw_cmd_ = 0;
    
    pos_ << 0.0, 0.0, 0.0;
    yaw_ = 0.0;

    reset();
}

void PosCtrl::reset(void)
{
    err_pos_last_ << 0.0, 0.0, 0.0;
    err_pos_integrate_ << 0.0, 0.0, 0.0;
    err_yaw_last_ = 0.0;
    err_yaw_integrate_ = 0.0;
}

void PosCtrl::setPosCtrlMode(const int &mode)
{
    mode_ = mode;
}

void PosCtrl::setPosCtrlRate(const double &rate)
{
    rate_ = rate;
}

void PosCtrl::setPosCtrlParam(const Eigen::Vector4d &kp,
                              const Eigen::Vector4d &ki,
                              const Eigen::Vector4d &kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PosCtrl::setPositionState(const Eigen::Vector3d &position,
                               const double &yaw)
{
    pos_ = position;
    yaw_ = yaw;
}

void PosCtrl::updateVelocityCmd(const Eigen::Vector3d &des_pos,
                                const double &des_yaw,
                                const Eigen::Vector3d &vel_ff)
{
    //
    Eigen::Vector3d error_pos = des_pos - pos_;
    //Position Control x y
    double vxp = 0.0, vxi = 0.0, vxd = 0.0, vx = 0.0;
    double vyp = 0.0, vyi = 0.0, vyd = 0.0, vy = 0.0;
    vxp = kp_[0] * error_pos[0];
    vyp = kp_[1] * error_pos[1];

    err_pos_integrate_[0] += error_pos[0];
    err_pos_integrate_[1] += error_pos[1];
    limit(err_pos_integrate_[0], XY_I_Err_Satur);
    limit(err_pos_integrate_[1], XY_I_Err_Satur);
    vxi = ki_[0] * err_pos_integrate_[0];
    vyi = ki_[1] * err_pos_integrate_[1];

    vxd = kd_[0] * (error_pos[0] - err_pos_last_[0]) * rate_;
    vyd = kd_[1] * (error_pos[1] - err_pos_last_[1]) * rate_;
    limit(vxd, XY_D_Satur);
    limit(vyd, XY_D_Satur);

    vx = vxp + vxi + vxd + vel_ff[0];
    vy = vyp + vyi + vyd + vel_ff[1];
    limit(vx, vy, XY_Satur);

    vel_cmd_[0] = vx;
    vel_cmd_[1] = vy;
    
    //Position Control z
    double vz = 0.0;
    vz = kp_[2] * error_pos[2];
    limit(vz, Z_Satur);

    vel_cmd_[2] = vz;
    
    //
    double error_yaw = (des_yaw - yaw_)* rad2deg;
    //Position Control Yaw
    yaw_cmd_ = yaw_;
    yawrate_cmd_ = 0.0;
    if (mode_ == 0) //mode 0 -> no yaw control
    {
        yaw_cmd_ = des_yaw; //set yaw_cmd_ as des_yaw to low level controller
    }
    else //mode 1 -> yaw control
    {
        double yawrate = 0.0, yawratep = 0.0, yawratei = 0.0, yawrated = 0.0;
        
        if (error_yaw < -180)
            error_yaw += 360;
        else if (error_yaw > 180)
            error_yaw -= 360;
        
        yawratep = kp_[3] * error_yaw;

        err_yaw_integrate_ += error_yaw;
        limit(err_yaw_integrate_, YAWRATE_I_Err_Satur);
        yawratei = ki_[3] * err_yaw_integrate_;

        yawrated = kd_[3] * (error_yaw - err_yaw_last_) * rate_;
        limit(yawrated, YAWRATE_D_Satur);

        yawrate = yawratep + yawratei + yawrated;
        limit(yawrate, YAWRATE_Satur);

        yawrate_cmd_ = yawrate * deg2rad;
    }

    //Update last error
    err_pos_last_ = error_pos;
    err_yaw_last_ = error_yaw;
}

const Eigen::Vector3d&
PosCtrl::getVelocityCmdENU(void)
{
    return vel_cmd_;
}

const Eigen::Vector3d&
PosCtrl::getVelocityCmdFLU(void)
{
    //Rotate only yaw
    vel_cmd_body_[0] = vel_cmd_[0] * cos(yaw_) + vel_cmd_[1] * sin(yaw_);
    vel_cmd_body_[1] = vel_cmd_[1] * cos(yaw_) - vel_cmd_[0] * sin(yaw_);
    vel_cmd_body_[2] = vel_cmd_[2];
    return vel_cmd_body_;
}

const double 
PosCtrl::getYawCmd(void)
{
    return yaw_cmd_;
}

const double 
PosCtrl::getYawrateCmd(void)
{
    return yawrate_cmd_;
}