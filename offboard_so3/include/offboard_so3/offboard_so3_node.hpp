#ifndef __OFFBOARD_SO3_NODE_H__
#define __OFFBOARD_SO3_NODE_H__

#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include <dynamic_reconfigure/server.h>
#include <offboard_so3/OffboardSo3ControllerConfig.h>
#include <std_srvs/SetBool.h>

#include <offboard_so3/pos_ctrl.hpp>
#include <offboard_so3/vel_ctrl.hpp>

//mav frame
int FRAME_LOCAL_NED = 1;
int FRAME_LOCAL_OFFSET_NED = 7;
int FRAME_BODY_NED = 8;
int FRAME_BODY_OFFSET_NED = 9;

//local ignore
int IGNORE_PX = 1;
int IGNORE_PY = 2;
int IGNORE_PZ = 4;
int IGNORE_VX = 8;
int IGNORE_VY = 16;
int IGNORE_VZ = 32;
int IGNORE_AFX = 64;
int IGNORE_AFY = 128;
int IGNORE_AFZ = 256;
int FORCE = 512;
int IGNORE_YAW = 1024;
int IGNORE_YAW_RATE = 2048;

//attitude ignore
int IGNORE_ROLL_RATE = 1;
int IGNORE_PITCH_RATE = 2;
int IGNORE_YAW_RATE_ATT = 4;
int IGNORE_THRUST = 64;
int IGNORE_ATTITUDE = 128;

//control musk
int POS_YAW_NED = 1;
int POS_YAW_FLU = 2;
int POS_YAWRATE_NED = 4;
int POS_YAWRATE_FLU = 8;
int VEL_YAW_ATT = 16;
int VEL_YAW_ACC = 32;
int VEL_YAWRATE_ATT = 64;
int VEL_YAWRATE_ACC = 128;

class OffboardSO3Node
{
public:
    void dynamicReconfigureCallback(offboard_so3::OffboardSo3ControllerConfig &config, uint32_t level);
    OffboardSO3Node(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const int &musk);
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    PosCtrl pos_controller_;
    VelCtrl vel_controller_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber plan_des_sub_;
    ros::Publisher local_pos_pub_;
    ros::Publisher local_pub_;
    ros::Publisher  att_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Timer posctrl_timer_, velctrl_timer_, status_timer_;

    ros::Time last_request_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    //des
    Eigen::Vector3d plan_position_;
    Eigen::Vector3d plan_velocity_;
    Eigen::Vector3d plan_acceleration_;
    double plan_yaw_;

    //cur
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    double current_yaw_;
    
    int musk_;
    int pos_mode_;
    int vel_mode_;
    double pos_ctrl_rate_;
    double vel_ctrl_rate_;
    
    //position control param
    Eigen::Vector4d pos_kp;
    Eigen::Vector4d pos_ki;
    Eigen::Vector4d pos_kd;
    double pos_xysatur;
    double pos_zsatur;
    double pos_yawratesatur;
    double pos_xy_dsatur;
    double pos_xy_i_errsatur;
    double pos_yaw_dsatur;
    double pos_yawrate_i_errsatur;

    //velocity control param
    double TWratio;
    Eigen::Vector4d vel_kp;
    Eigen::Vector4d vel_ki;
    Eigen::Vector4d vel_kd;
    Eigen::Vector3d vel_accsatur;
    double vel_rollsatur;
    double vel_pitchsatur;
    double vel_yawratesatur;
    Eigen::Vector3d vel_xyz_dsatur;
    Eigen::Vector3d vel_xyz_i_errsatur;
    double vel_yawrate_dsatur;
    double vel_yawrate_i_errsatur;

    //info count
    int vel_cnt;
    int att_cnt;
    int odom_cnt;
    
    //takeoff flag
    bool takeoff_;

    void state_callback(const mavros_msgs::State::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void vel_callback(const geometry_msgs::TwistStamped::ConstPtr &twist);
    void plan_des_callback(const mavros_msgs::PositionTarget::ConstPtr &plan);
    void posctrl_callback(const ros::TimerEvent &event);
    void velctrl_callback(const ros::TimerEvent &event);
    void status_Callback(const ros::TimerEvent &event);
    void velcmd_NED_publish(const Eigen::Vector3d &velcmd, const double &yawratecmd);
    void att_thr_cmd_publish(const Eigen::Vector4d &quatcmd, const double &thrcmd);
    void acc_yawrate_cmd_publish(const Eigen::Vector3d &acccmd, const double &yawratecmd);
    void take_off(void);
};

#endif

Eigen::Vector4d my_pos_kp;
double my_TWratio;
Eigen::Vector4d my_vel_kp;
Eigen::Vector4d my_vel_ki;
Eigen::Vector3d my_vel_xyz_i_errsatur;
double my_vel_yawrate_i_errsatur;
Eigen::Vector3d my_plan_pos;
Eigen::Vector3d my_plan_vel;
double my_plan_yaw;