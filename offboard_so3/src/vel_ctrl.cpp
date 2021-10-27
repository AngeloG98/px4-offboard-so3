#include <offboard_so3/vel_ctrl.hpp>

void VelCtrl::init(const int &mode,
                   const double &rate,
                   const double &TWratio,
                   const Eigen::Vector4d &kp,
                   const Eigen::Vector4d &ki,
                   const Eigen::Vector4d &kd,
                   const Eigen::Vector3d &accsatur,
                   const double &rollsatur,
                   const double &pitchsatur,
                   const double &yawratesatur,
                   const Eigen::Vector3d &xyz_dsatur,
                   const Eigen::Vector3d &xyz_i_errsatur,
                   const double &yawrate_dsatur,
                   const double &yawrate_i_errsatur)
{
    mode_ = mode;
    rate_ = rate;
    T_W_ratio_ = TWratio;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    Acc_Satur = accsatur;
    Roll_Satur = rollsatur;
    Pitch_Satur = pitchsatur;
    YAWRATE_Satur = yawratesatur;
    XYZ_D_Satur = xyz_dsatur;
    XYZ_I_Err_Satur = xyz_i_errsatur;
    YAWRATE_D_Satur = yawrate_dsatur;
    YAWRATE_I_Err_Satur = yawrate_i_errsatur;

    acc_cmd_ << 0.0, 0.0, 0.0;
    f_cmd_ << 0.0, 0.0, 9.8;
    RotMat_cmd_ << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    Quat_cmd_ << 1.0, 0.0, 0.0, 0.0;
    Euler_cmd_ << 0.0, 0.0, 0.0;
    yaw_cmd_ = 0;
    yawrate_cmd_ = 0;
    thr_cmd_ = 0;

    g_ << 0.0, 0.0, -9.8;
    vel_ << 0.0, 0.0, 0.0;
    yaw_ = 0.0;

    reset();
}

void VelCtrl::reset(void)
{
    err_vel_last_ << 0.0, 0.0, 0.0;
    err_vel_integrate_ << 0.0, 0.0, 0.0;
    err_yaw_last_ = 0;
    err_yaw_integrate_ = 0;
}

void VelCtrl::setVelCtrlMode(const int &mode)
{
    mode_ = mode;
}

void VelCtrl::setVelCtrlRate(const double &rate)
{
    rate_ = rate;
}

void VelCtrl::setModelParam(const double &T_W_ratio)
{
    T_W_ratio_ = T_W_ratio;
}

void VelCtrl::setVelCtrlParam(const Eigen::Vector4d &kp,
                              const Eigen::Vector4d &ki,
                              const Eigen::Vector4d &kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void VelCtrl::setVelocityState(const Eigen::Vector3d &velocity,
                               const double &yaw)
{
    vel_ = velocity;
    yaw_ = yaw;
}

Eigen::Vector4d VelCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw)
{
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

void VelCtrl::updateAttitudeCmd(const Eigen::Vector3d &des_vel, const double &des_yaw,
                                const Eigen::Vector3d &acc_ff)
{
    //
    Eigen::Vector3d error_vel = des_vel - vel_;
    //Velocity Control x y z
    double acc_d;
    for (int i = 0; i < 3; i++)
    {
        err_vel_integrate_[i] += error_vel[i];
        limit(err_vel_integrate_[i], XYZ_I_Err_Satur[i]);
        acc_d = kd_[i] * (error_vel[i] - err_vel_last_[i]) * rate_;
        limit(acc_d, XYZ_D_Satur[i]);
        acc_cmd_[i] = kp_[i] * error_vel[i] \
                    + ki_[i] * err_vel_integrate_[i] \
                    + acc_d \
                    + acc_ff[i];
    }
    ROS_INFO("acc zp%f", kp_[2]);
    ROS_INFO("acc zi%f", ki_[2]);
    ROS_INFO("T_W_ratio_ = %.2f ", T_W_ratio_);
    ROS_INFO("XYZ_I_Err_Satur = %.2f ", XYZ_I_Err_Satur[2]);
    // ROS_INFO("acc z: p/ i/ d %f %f %f", kp_[2], ki_[2], 0.0);
    f_cmd_ = acc_cmd_ - g_;
    limit(f_cmd_[0], f_cmd_[1], f_cmd_[2], T_W_ratio_ * abs(g_[2]));
    thr_cmd_ = std::max(0.0, std::min(1.0, f_cmd_.norm() / (T_W_ratio_ * abs(g_[2]))));

    //
    double error_yaw = (des_yaw - yaw_)* rad2deg;
    //Velocity Control Yaw
    yaw_cmd_ = yaw_;
    yawrate_cmd_ = 0.0;
    if (mode_ == 0) //mode_ == 0 -> no yaw control
    {
        yaw_cmd_ = des_yaw; //set yaw_cmd_ as des_yaw to low level controller
    }
    else //mode_ == 1 -> yaw control
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
    err_vel_last_ = error_vel;
    err_yaw_last_ = error_yaw;

    //
    if (mode_ == 0) //no yaw control, use yaw_cmd_(also des_yaw)
    {
        Quat_cmd_ = acc2quaternion(f_cmd_, yaw_cmd_);
    }
    else //mode_ == 2
    {
        Quat_cmd_ = acc2quaternion(f_cmd_, yaw_);
    }
}


const Eigen::Vector3d&
VelCtrl::getAttitudeCmdAcc(void)
{
    return acc_cmd_;
}
const Eigen::Matrix3d&
VelCtrl::getAttitudeCmdRotMat(void)
{
    RotMat_cmd_ = quat2RotMatrix(Quat_cmd_);
    return RotMat_cmd_;
}

const Eigen::Vector4d&
VelCtrl::getAttitudeCmdQuat(void)
{
    return Quat_cmd_;
}

const Eigen::Vector3d&
VelCtrl::getAttitudeCmdEuler(void)
{
    Euler_cmd_ = quat2EulerAngle(Quat_cmd_);
    return Euler_cmd_;
}

const double&
VelCtrl::getYawrateCmd(void)
{
    return yawrate_cmd_;
}

const double&
VelCtrl::getThrottleCmd(void)
{
    return thr_cmd_;
}