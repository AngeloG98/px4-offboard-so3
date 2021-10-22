#include <offboard_so3/offboard_so3_node.hpp>


OffboardSO3Node::OffboardSO3Node(const ros::NodeHandle &nh, const int &musk, const double &ctrl_rate)
        :nh_(nh), musk_(musk), ctrl_rate_(ctrl_rate)
{
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardSO3Node::state_callback,this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &OffboardSO3Node::odom_callback,this);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    local_att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    posctrl_timer_ = nh_.createTimer(ros::Duration(1/ctrl_rate_), &OffboardSO3Node::posctrl_callback,this);
    velctrl_timer_ = nh_.createTimer(ros::Duration(1/ctrl_rate_), &OffboardSO3Node::velctrl_callback,this);
    status_timer_ = nh_.createTimer(ros::Duration(0.05), &OffboardSO3Node::status_Callback,this);
    
    last_request_ = ros::Time::now();
    vel_cnt = 0;
    att_cnt = 0;
    odom_cnt = 0;

    pos_controller_.setPosCtrlMode(1);
    pos_controller_.setPosCtrlRate(ctrl_rate_);

    vel_controller_.setVelCtrlMode(0);
    vel_controller_.setVelCtrlRate(ctrl_rate_);
    // vel_controller_.setModelParam(1,20);

    Eigen::Vector4d Kp(1,1,1,1);
    Eigen::Vector4d Ki(0.5,0.5,0,0);
    Eigen::Vector4d Kd(0,0,0,0);
    pos_controller_.setPosCtrlParam(Kp, Ki, Kd);
}

void
OffboardSO3Node::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_ = *msg;
}

void
OffboardSO3Node::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    const Eigen::Vector3d position(odom->pose.pose.position.x,
                                   odom->pose.pose.position.y,
                                   odom->pose.pose.position.z);
    const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                   odom->twist.twist.linear.y,
                                   odom->twist.twist.linear.z);
    current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
    pos_controller_.setPositionState(position, current_yaw_);
    vel_controller_.setVelocityState(velocity, current_yaw_);
    if (odom_cnt % 15 == 0)
    {
        ROS_INFO("current position : %f %f %f %f", position[0],position[1],position[2],current_yaw_*rad2deg);
        ROS_INFO("current velocity : %f %f %f", velocity[0],velocity[1],velocity[2]);
    }
    odom_cnt++;
}

void
OffboardSO3Node::posctrl_callback(const ros::TimerEvent &event)
{
    if (current_state_.armed && current_state_.mode == "OFFBOARD")
    {
        // Eigen::Vector3d des_pos(1, 1, 1);
        // double des_yaw = 0;
        // Eigen::Vector3d vel_ff(0, 0, 0);
        // pos_controller_.updateVelocityCmd(des_pos, des_yaw, vel_ff);
        // velcmd_NED_publish();
    }
}

void
OffboardSO3Node::velctrl_callback(const ros::TimerEvent &event)
{
    // if (current_state_.armed && current_state_.mode == "OFFBOARD")
    // if (current_state_.armed)
    if (1)
    {
        Eigen::Vector3d des_vel(0, 0, 1);
        double des_yaw = 0;
        Eigen::Vector3d acc_ff(0, 0, 0);
        vel_controller_.updateAttitudeCmd(des_vel, des_yaw, acc_ff);
        att_thr_cmd_publish(vel_controller_.getAttitudeCmdQuat(),vel_controller_.getThrottleCmd());
    }
}

void OffboardSO3Node::velcmd_NED_publish(const Eigen::Vector3d &velcmd, const double &yawratecmd)
{
    geometry_msgs::TwistStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.seq = 1;
    msg.twist.linear.x = velcmd[0];
    msg.twist.linear.y = velcmd[1];
    msg.twist.linear.z = velcmd[2];
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = yawratecmd;
    local_vel_pub_.publish(msg);
    if (vel_cnt % 50 == 0)
    {
        ROS_INFO("velocity cmd : %f %f %f %f", msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z * rad2deg);
    }
    vel_cnt++;
}

void
OffboardSO3Node::att_thr_cmd_publish(const Eigen::Vector4d &quatcmd, const double &thrcmd)
{
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "att_f";
    msg.body_rate.x = 0;
    msg.body_rate.y = 0;
    msg.body_rate.z = 0;
    msg.type_mask = 7;  // Ignore rate messages
    msg.orientation.w = quatcmd[0];
    msg.orientation.x = quatcmd[1];
    msg.orientation.y = quatcmd[2];
    msg.orientation.z = quatcmd[3];
    msg.thrust = thrcmd;
    local_att_pub_.publish(msg);
    if (att_cnt % 50 == 0)
    {
        ROS_INFO("att and thr cmd : %f %f %f %f %f", \
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.thrust);
    }
    att_cnt++;
}

void
OffboardSO3Node::status_Callback(const ros::TimerEvent &event)
{
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        ROS_INFO("Connecting...");
    }
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(2.0))) 
    {
        if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) 
        {
            ROS_INFO("Offboard enabled");
        }
        last_request_ = ros::Time::now();
    } 
    else {
        if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(0.1))) 
        {
            ROS_INFO("Sending arming command..."); 
            if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
            {
              ROS_INFO("Vehicle armed");
            }
            last_request_ = ros::Time::now();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_so3_node");
    ros::NodeHandle nh;
    int musk = 1;
    double ctrl_rate = 100;
    OffboardSO3Node OffboardSO3Controller(nh, musk, ctrl_rate);

    ros::spin();
    return 0;
}