#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <offboard_so3/pos_ctrl.hpp>
#include <offboard_so3/vel_ctrl.hpp>


class OffboardSO3Node
{
public:
    OffboardSO3Node(const ros::NodeHandle &nh, const int &musk, const double &ctrl_rate);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    PosCtrl pos_controller_;
    VelCtrl vel_controller_;

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber plan_des_sub_;
    ros::Publisher local_pos_pub_;
    ros::Publisher local_vel_pub_;
    ros::Publisher  local_att_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Timer posctrl_timer_, velctrl_timer_, status_timer_;
    ros::Time last_request_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    double current_yaw_;
    double ctrl_rate_;
    int musk_;
    int vel_cnt;
    int att_cnt;
    int odom_cnt;
    bool takeoff_;

    void state_callback(const mavros_msgs::State::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void vel_callback(const geometry_msgs::TwistStamped::ConstPtr &twist);
    void posctrl_callback(const ros::TimerEvent &event);
    void velctrl_callback(const ros::TimerEvent &event);
    void status_Callback(const ros::TimerEvent &event);
    void velcmd_NED_publish(const Eigen::Vector3d &velcmd, const double &yawratecmd);
    void att_thr_cmd_publish(const Eigen::Vector4d &quatcmd, const double &thrcmd);
    void take_off(void);
};