#include <offboard_so3/offboard_so3_node.hpp>


OffboardSO3Node::OffboardSO3Node(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const int &musk)
        :nh_(nh), nh_private_(nh_private), musk_(musk)
{    
    //set controller mode
    if (musk_ == POS_YAW_NED ||
        musk_ == POS_YAW_FLU ||
        musk_ == VEL_YAW_ATT ||
        musk_ == VEL_YAW_ACC ||
        musk_ == POS_YAW_NED + VEL_YAW_ATT ||
        musk_ == POS_YAW_NED + VEL_YAW_ACC)
    {
        pos_mode_ = 0;
        vel_mode_ = 0;
    }
    else if (musk_ == POS_YAWRATE_NED ||
            musk_ == POS_YAWRATE_FLU)
    {
        pos_mode_ = 1;
        vel_mode_ = 0;
    }
    else if(musk_ == POS_YAW_NED + VEL_YAWRATE_ATT ||
            musk_ == POS_YAW_NED + VEL_YAWRATE_ACC ||
            musk_ == VEL_YAWRATE_ATT ||
            musk_ == VEL_YAWRATE_ACC)
    {
        pos_mode_ = 0;
        vel_mode_ = 1;
    }
    else
    {
        ROS_WARN("Invalid musk, use position control with yaw control.");
        musk_ = POS_YAWRATE_NED;
        pos_mode_ = 1;
        vel_mode_ = 0;
    } 

    //set controller parameters
    nh_private_.param("", pos_ctrl_rate_, 50);
    nh_private_.param("", vel_ctrl_rate_, 100);

    nh_private_.param("", pos_kp[0], 1.0);
    nh_private_.param("", pos_kp[1], 1.0);
    nh_private_.param("", pos_kp[2], 1.0);
    nh_private_.param("", pos_kp[3], 1.0);

    nh_private_.param("", pos_ki[0], 0.0);
    nh_private_.param("", pos_ki[1], 0.0);
    nh_private_.param("", pos_ki[2], 0.0);
    nh_private_.param("", pos_ki[3], 0.0);

    nh_private_.param("", pos_kd[0], 0.0);
    nh_private_.param("", pos_kd[1], 0.0);
    nh_private_.param("", pos_kd[2], 0.0);
    nh_private_.param("", pos_kd[3], 0.0);

    nh_private_.param("", pos_xysatur, 3.0);
    nh_private_.param("", pos_zsatur, 2.0);
    nh_private_.param("", pos_yawratesatur, 30.0);
    nh_private_.param("", pos_xy_dsatur, 0.5);
    nh_private_.param("", pos_xy_i_errsatur, 0.1);
    nh_private_.param("", pos_yaw_dsatur, 5.0);
    nh_private_.param("", pos_yawrate_i_errsatur, 10.0);
    
    nh_private_.param("", vel_fwratio, 17.5);
    
    nh_private_.param("", vel_kp[0], 2.0);
    nh_private_.param("", vel_kp[1], 2.0);
    nh_private_.param("", vel_kp[2], 1.5);
    nh_private_.param("", vel_kp[3], 1.0);

    nh_private_.param("", vel_ki[0], 0.1);
    nh_private_.param("", vel_ki[1], 0.1);
    nh_private_.param("", vel_ki[2], 0.2);
    nh_private_.param("", vel_ki[3], 0.0);

    nh_private_.param("", vel_kd[0], 0.0);
    nh_private_.param("", vel_kd[1], 0.0);
    nh_private_.param("", vel_kd[2], 0.0);
    nh_private_.param("", vel_kd[3], 0.0);

    nh_private_.param("", vel_accsatur[0], 2.0);
    nh_private_.param("", vel_accsatur[1], 2.0);
    nh_private_.param("", vel_accsatur[2], 2.0);
    nh_private_.param("", vel_pitchsatur, 30.0);
    nh_private_.param("", vel_rollsatur, 60.0);
    nh_private_.param("", vel_yawratesatur, 30.0);
    nh_private_.param("", vel_xyz_dsatur[0], 0.1);
    nh_private_.param("", vel_xyz_dsatur[1], 0.1);
    nh_private_.param("", vel_xyz_dsatur[2], 0.1);
    nh_private_.param("", vel_xyz_i_errsatur[0], 0.1);
    nh_private_.param("", vel_xyz_i_errsatur[1], 0.1);
    nh_private_.param("", vel_xyz_i_errsatur[2], 0.3);
    nh_private_.param("", vel_yawrate_dsatur, 5.0);
    nh_private_.param("", vel_yawrate_i_errsatur, 10.0);

    pos_controller_.init(pos_mode_, pos_ctrl_rate_,
                         pos_kp, pos_ki, pos_kd,
                         pos_xysatur, pos_zsatur, pos_yawratesatur,
                         pos_xy_dsatur, pos_xy_i_errsatur,
                         pos_yaw_dsatur, pos_yawrate_i_errsatur);
    
    vel_controller_.init(vel_mode_, vel_ctrl_rate_,
                         vel_fwratio,
                         vel_kp, vel_ki, vel_kd,
                         vel_accsatur,
                         vel_pitchsatur, vel_rollsatur, vel_yawratesatur,
                         vel_xyz_dsatur, vel_xyz_i_errsatur,
                         vel_yawrate_dsatur, vel_yawrate_i_errsatur);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    local_att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardSO3Node::state_callback,this);
    // odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &OffboardSO3Node::odom_callback,this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &OffboardSO3Node::pose_callback,this);
    vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &OffboardSO3Node::vel_callback,this);
    
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    posctrl_timer_ = nh_.createTimer(ros::Duration(1/ctrl_rate_), &OffboardSO3Node::posctrl_callback,this);
    velctrl_timer_ = nh_.createTimer(ros::Duration(1/ctrl_rate_), &OffboardSO3Node::velctrl_callback,this);
    status_timer_ = nh_.createTimer(ros::Duration(0.05), &OffboardSO3Node::status_Callback,this);
    

    last_request_ = ros::Time::now();
    
    vel_cnt = 0;
    att_cnt = 0;
    odom_cnt = 0;

    current_position_ << 0.0, 0.0, 0.0;
    current_velocity_ << 0.0, 0.0, 0.0;
    current_yaw_ = 0;

    takeoff_ = false;
}

void
OffboardSO3Node::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_ = *msg;
}

void
OffboardSO3Node::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    current_position_ = Eigen::Vector3d(odom->pose.pose.position.x,
                                        odom->pose.pose.position.y,
                                        odom->pose.pose.position.z);
    current_velocity_ = Eigen::Vector3d(odom->twist.twist.linear.x,
                                        odom->twist.twist.linear.y,
                                        odom->twist.twist.linear.z);
    current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
    pos_controller_.setPositionState(current_position_, current_yaw_);
    vel_controller_.setVelocityState(current_velocity_, current_yaw_);
    // if (odom_cnt % 15 == 0)
    // {
    //     ROS_INFO("current position : %f %f %f %f", current_position_[0],current_position_[1],current_position_[2],current_yaw_*rad2deg);
    //     ROS_INFO("current velocity : %f %f %f", current_velocity_[0],current_velocity_[1],current_velocity_[2]);
    // }
    // odom_cnt++;
}

void
OffboardSO3Node::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    current_position_ = Eigen::Vector3d(pose->pose.position.x,
                                        pose->pose.position.y,
                                        pose->pose.position.z);
    current_yaw_ = tf::getYaw(pose->pose.orientation);
    pos_controller_.setPositionState(current_position_, current_yaw_);
}

void
OffboardSO3Node::vel_callback(const geometry_msgs::TwistStamped::ConstPtr &twist)
{
    current_velocity_ = Eigen::Vector3d(twist->twist.linear.x,
                                        twist->twist.linear.y,
                                        twist->twist.linear.z);
    vel_controller_.setVelocityState(current_velocity_, current_yaw_);
}

void
OffboardSO3Node::posctrl_callback(const ros::TimerEvent &event)
{
    if (takeoff_)
    {
        // Eigen::Vector3d des_pos(2, 0, 2);
        // double des_yaw = 0;
        // Eigen::Vector3d vel_ff(0, 0, 0);
        // pos_controller_.updateVelocityCmd(des_pos, des_yaw, vel_ff);
        // velcmd_NED_publish(pos_controller_.getVelocityCmdENU(),pos_controller_.getYawrateCmd());
        Eigen::Vector3d vel(0, 0, 0);
        double yaw = 0.0;
        velcmd_NED_publish(vel, yaw);
    }
}

void
OffboardSO3Node::velctrl_callback(const ros::TimerEvent &event)
{
    if (takeoff_)
    {
        // Eigen::Vector3d des_vel(0, 0, 0);
        // double des_yaw = 0;
        // Eigen::Vector3d acc_ff(0, 0, 0);
        // vel_controller_.updateAttitudeCmd(des_vel, des_yaw, acc_ff);
        // att_thr_cmd_publish(vel_controller_.getAttitudeCmdQuat(),vel_controller_.getThrottleCmd());
    }
}

void OffboardSO3Node::velcmd_NED_publish(const Eigen::Vector3d &velcmd, const double &yawratecmd)
{
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "vel_ned";
    msg.coordinate_frame = FRAME_LOCAL_NED;
    msg.type_mask = IGNORE_PX + IGNORE_PY + IGNORE_PZ + IGNORE_AFX + IGNORE_AFY \
                    + IGNORE_AFZ + FORCE + IGNORE_YAW;  //ignore pos acc yaw messages
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 0;
    msg.velocity.x = velcmd[0];
    msg.velocity.y = velcmd[1];
    msg.velocity.z = velcmd[2];
    msg.acceleration_or_force.x = 0;
    msg.acceleration_or_force.y = 0;
    msg.acceleration_or_force.z = 0;
    msg.yaw = 0;
    msg.yaw_rate = yawratecmd;
    local_vel_pub_.publish(msg);
    if (vel_cnt % 50 == 0)
    {
        ROS_INFO("velocity cmd : %f %f %f %f", msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.yaw_rate * rad2deg);
    }
    vel_cnt++;
    if (odom_cnt % 50 == 0)
    {
        ROS_INFO("current position : %f %f %f %f", current_position_[0],current_position_[1],current_position_[2],current_yaw_*rad2deg);
        ROS_INFO("current velocity : %f %f %f", current_velocity_[0],current_velocity_[1],current_velocity_[2]);
    }
    odom_cnt++;
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
    msg.type_mask = IGNORE_ROLL_RATE + IGNORE_PITCH_RATE + IGNORE_YAW_RATE;  // Ignore rate messages
    msg.orientation.w = quatcmd[0];
    msg.orientation.x = quatcmd[1];
    msg.orientation.y = quatcmd[2];
    msg.orientation.z = quatcmd[3];
    msg.thrust = thrcmd;
    local_att_pub_.publish(msg);
    Eigen::Vector3d euler = vel_controller_.getAttitudeCmdEuler();
    if (att_cnt % 50 == 0)
    {
        ROS_INFO("att and thr cmd : %f %f %f %f %f", \
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.thrust);
        ROS_INFO("att cmd : %f %f %f ",euler[0], euler[1], euler[2]);
    }
    att_cnt++;
    if (odom_cnt % 50 == 0)
    {
        ROS_INFO("current position : %f %f %f %f", current_position_[0],current_position_[1],current_position_[2],current_yaw_*rad2deg);
        ROS_INFO("current velocity : %f %f %f", current_velocity_[0],current_velocity_[1],current_velocity_[2]);
    }
    odom_cnt++;
}

void
OffboardSO3Node::status_Callback(const ros::TimerEvent &event)
{
    while(ros::ok() && !current_state_.connected)
    {
        ros::spinOnce();
        ROS_INFO("Connecting...");
    }
    take_off();
}

void
OffboardSO3Node::take_off(void)
{
    //simulation
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
        else
        {   
            //set init pose
            double init_yaw = 0;
            geometry_msgs::PoseStamped init_pose;
            init_pose.pose.position.x = 0;
            init_pose.pose.position.y = 0;
            init_pose.pose.position.z = 1.5;
            init_pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_yaw);
            Eigen::Vector3d init_position(init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z);
            if (!takeoff_)
            {
                ROS_INFO("Taking off...");
                local_pos_pub_.publish(init_pose);
                if ((current_position_ - init_position).norm() < 0.1 && abs(current_yaw_ - init_yaw) < 0.15)
                {
                    takeoff_ = true;
                }
            }            
        }
    }
    
    //realflight
    //set init pose
    // double init_yaw = 0;
    // geometry_msgs::PoseStamped init_pose;
    // init_pose.pose.position.x = 0;
    // init_pose.pose.position.y = 0;
    // init_pose.pose.position.z = 1.5;
    // init_pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_yaw);
    // Eigen::Vector3d init_position(init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z);
    
    // if (current_state_.armed && current_state_.mode == "OFFBOARD")
    // {
    //     if (!takeoff_)
    //     {
    //         ROS_INFO("Taking off...");
    //         local_pos_pub_.publish(init_pose);
    //         if ((current_position_ - init_position).norm() < 0.2 && abs(current_yaw_ - init_yaw) < 0.15)
    //         {
    //             takeoff_ = true;
    //         }
    //     }
    // }
    // else
    // {
    //     ROS_INFO("Sending take off command...");
    //     local_pos_pub_.publish(init_pose);
    //     takeoff_ = false;
    //     pos_controller_.reset();
    //     vel_controller_.reset();
    // }
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