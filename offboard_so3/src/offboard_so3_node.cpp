#include <offboard_so3/offboard_so3_node.hpp>


OffboardSO3Node::OffboardSO3Node(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const int &musk)
        :nh_(nh), nh_private_(nh_private), musk_(musk)
{    
    //set controller mode
    if (musk_ == VEL_YAW_ATT ||
        musk_ == POS_YAW_NED + VEL_YAW_ATT)
    {
        pos_mode_ = 0;
        vel_mode_ = 0;
    }
    else if (musk_ == POS_YAWRATE_NED)
    {
        pos_mode_ = 1;
        vel_mode_ = 0;
    }
    else if(musk_ == VEL_YAWRATE_ACC ||
            musk_ == POS_YAW_NED + VEL_YAWRATE_ACC)
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
    nh_private_.param("rate/pos_rate", pos_ctrl_rate_, 50.0);
    nh_private_.param("rate/vel_rate", vel_ctrl_rate_, 100.0);

    nh_private_.param("gain/pos_kp/x", pos_kp[0], 1.0);
    nh_private_.param("gain/pos_kp/y", pos_kp[1], 1.0);
    nh_private_.param("gain/pos_kp/z", pos_kp[2], 1.0);
    nh_private_.param("gain/pos_kp/yaw", pos_kp[3], 1.0);

    nh_private_.param("gain/pos_ki/x", pos_ki[0], 0.0);
    nh_private_.param("gain/pos_ki/y", pos_ki[1], 0.0);
    nh_private_.param("gain/pos_ki/z", pos_ki[2], 0.0);
    nh_private_.param("gain/pos_ki/yaw", pos_ki[3], 0.0);

    nh_private_.param("gain/pos_kd/x", pos_kd[0], 0.0);
    nh_private_.param("gain/pos_kd/y", pos_kd[1], 0.0);
    nh_private_.param("gain/pos_kd/z", pos_kd[2], 0.0);
    nh_private_.param("gain/pos_kd/yaw", pos_kd[3], 0.0);

    nh_private_.param("satur/pos/vel_cmd/xy", pos_xysatur, 5.0);
    nh_private_.param("satur/pos/vel_cmd/z", pos_zsatur, 2.0);
    nh_private_.param("satur/pos/vel_cmd/yawrate", pos_yawratesatur, 45.0);
    nh_private_.param("satur/pos/d/xy", pos_xy_dsatur, 0.1);
    nh_private_.param("satur/pos/i_err/yawrate", pos_xy_i_errsatur, 0.1);
    nh_private_.param("satur/pos/d/xy", pos_yaw_dsatur, 1.0);
    nh_private_.param("satur/pos/i_err/yawrate", pos_yawrate_i_errsatur, 1.0);
    
    nh_private_.param("twratio", TWratio, 1.85);

    nh_private_.param("gain/vel_kp/x", vel_kp[0], 2.0);
    nh_private_.param("gain/vel_kp/y", vel_kp[1], 2.0);
    nh_private_.param("gain/vel_kp/z", vel_kp[2], 1.5);
    nh_private_.param("gain/vel_kp/yaw", vel_kp[3], 1.0);

    nh_private_.param("gain/vel_ki/x", vel_ki[0], 0.1);
    nh_private_.param("gain/vel_ki/y", vel_ki[1], 0.1);
    nh_private_.param("gain/vel_ki/z", vel_ki[2], 0.1);
    nh_private_.param("gain/vel_ki/yaw", vel_ki[3], 0.0);

    nh_private_.param("gain/vel_kd/x", vel_kd[0], 0.0);
    nh_private_.param("gain/vel_kd/y", vel_kd[1], 0.0);
    nh_private_.param("gain/vel_kd/z", vel_kd[2], 0.0);
    nh_private_.param("gain/vel_kd/yaw", vel_kd[3], 0.0);

    nh_private_.param("satur/vel/acc_cmd/x", vel_accsatur[0], 2.0);
    nh_private_.param("satur/vel/acc_cmd/y", vel_accsatur[1], 2.0);
    nh_private_.param("satur/vel/acc_cmd/z", vel_accsatur[2], 2.0);
    nh_private_.param("satur/vel/att_cmd/roll", vel_rollsatur, 60.0);
    nh_private_.param("satur/vel/att_cmd/pitch", vel_pitchsatur, 30.0);
    nh_private_.param("satur/vel/att_cmd/yawrate", vel_yawratesatur, 30.0);
    nh_private_.param("satur/vel/d/x", vel_xyz_dsatur[0], 0.1);
    nh_private_.param("satur/vel/d/y", vel_xyz_dsatur[1], 0.1);
    nh_private_.param("satur/vel/d/z", vel_xyz_dsatur[2], 0.1);
    nh_private_.param("satur/vel/i_err/x", vel_xyz_i_errsatur[0], 0.1);
    nh_private_.param("satur/vel/i_err/y", vel_xyz_i_errsatur[1], 0.1);
    nh_private_.param("satur/vel/i_err/z", vel_xyz_i_errsatur[2], 0.1);
    nh_private_.param("satur/vel/d/yawrate", vel_yawrate_dsatur, 1.0);
    nh_private_.param("satur/vel/i_err/yawrate", vel_yawrate_i_errsatur, 1.0);

    pos_controller_.init(pos_mode_, pos_ctrl_rate_,
                         pos_kp, pos_ki, pos_kd,
                         pos_xysatur, pos_zsatur, pos_yawratesatur,
                         pos_xy_dsatur, pos_xy_i_errsatur,
                         pos_yaw_dsatur, pos_yawrate_i_errsatur);

    vel_controller_.init(vel_mode_, vel_ctrl_rate_,
                         TWratio,
                         vel_kp, vel_ki, vel_kd,
                         vel_accsatur,
                         vel_rollsatur, vel_pitchsatur, vel_yawratesatur,
                         vel_xyz_dsatur, vel_xyz_i_errsatur,
                         vel_yawrate_dsatur, vel_yawrate_i_errsatur);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardSO3Node::state_callback,this);
    // odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &OffboardSO3Node::odom_callback,this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &OffboardSO3Node::pose_callback,this);
    vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &OffboardSO3Node::vel_callback,this);
    plan_des_sub_ = nh_.subscribe<mavros_msgs::PositionTarget>("/plan/trajectory", 10, &OffboardSO3Node::plan_des_callback,this);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    posctrl_timer_ = nh_.createTimer(ros::Duration(1/pos_ctrl_rate_), &OffboardSO3Node::posctrl_callback,this);
    velctrl_timer_ = nh_.createTimer(ros::Duration(1/vel_ctrl_rate_), &OffboardSO3Node::velctrl_callback,this);
    status_timer_ = nh_.createTimer(ros::Duration(0.05), &OffboardSO3Node::status_Callback,this);
    

    last_request_ = ros::Time::now();
    
    vel_cnt = 0;
    att_cnt = 0;
    odom_cnt = 0;

    plan_position_ << 0.0, 0.0, 0.0;
    plan_velocity_ << 0.0, 0.0, 0.0;
    plan_acceleration_ << 0.0, 0.0, 0.0;
    plan_yaw_ = 0.0;

    current_position_ << 0.0, 0.0, 0.0;
    current_velocity_ << 0.0, 0.0, 0.0;
    current_yaw_ = 0.0;

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
OffboardSO3Node::plan_des_callback(const mavros_msgs::PositionTarget::ConstPtr &plan)
{

}

void
OffboardSO3Node::posctrl_callback(const ros::TimerEvent &event)
{
    plan_position_ = my_plan_pos;
    plan_velocity_ = my_plan_vel;
    plan_yaw_ = my_plan_yaw;

    pos_kp = my_pos_kp;
    pos_controller_.setPosCtrlParam(pos_kp, pos_ki, pos_kd);
    if (takeoff_)
    {  
        if (musk_ == VEL_YAW_ATT ||
            musk_ == POS_YAW_NED + VEL_YAW_ATT)
        {
            pos_controller_.updateVelocityCmd(plan_position_, plan_yaw_, plan_velocity_);
        }
        else if (musk_ == POS_YAWRATE_NED)
        {
            pos_controller_.updateVelocityCmd(plan_position_, plan_yaw_, plan_velocity_);
            velcmd_NED_publish(pos_controller_.getVelocityCmdENU(),pos_controller_.getYawrateCmd());
        }
        else if(musk_ == VEL_YAWRATE_ACC ||
                musk_ == POS_YAW_NED + VEL_YAWRATE_ACC)
        {
            pos_controller_.updateVelocityCmd(plan_position_, plan_yaw_, plan_velocity_);
        }
    }
}

void
OffboardSO3Node::velctrl_callback(const ros::TimerEvent &event)
{
    plan_velocity_ = my_plan_vel;
    plan_yaw_ = my_plan_yaw;

    TWratio = my_TWratio;
    vel_kp = my_vel_kp;
    vel_ki = my_vel_ki;
    vel_controller_.XYZ_I_Err_Satur = my_vel_xyz_i_errsatur;
    vel_controller_.YAWRATE_I_Err_Satur = my_vel_yawrate_i_errsatur;
    vel_controller_.setModelParam(TWratio);
    vel_controller_.setVelCtrlParam(vel_kp, vel_ki, vel_kd);
    if (takeoff_)
    {
        if (musk_ == VEL_YAW_ATT)
        {
            vel_controller_.updateAttitudeCmd(plan_velocity_, plan_yaw_, plan_acceleration_);
            att_thr_cmd_publish(vel_controller_.getAttitudeCmdQuat(),vel_controller_.getThrottleCmd());
        }
        else if (musk_ == POS_YAW_NED + VEL_YAW_ATT)
        {
            vel_controller_.updateAttitudeCmd(pos_controller_.getVelocityCmdENU(), plan_yaw_, plan_acceleration_);
            att_thr_cmd_publish(vel_controller_.getAttitudeCmdQuat(),vel_controller_.getThrottleCmd());
        }
        else if (musk_ == POS_YAWRATE_NED)
        {
            //
        }
        else if(musk_ == VEL_YAWRATE_ACC)
        {
            vel_controller_.updateAttitudeCmd(plan_velocity_, plan_yaw_, plan_acceleration_);
            acc_yawrate_cmd_publish(vel_controller_.getAttitudeCmdAcc(),vel_controller_.getYawrateCmd());
        }
        else if (musk_ == POS_YAW_NED + VEL_YAWRATE_ACC)
        {
            vel_controller_.updateAttitudeCmd(pos_controller_.getVelocityCmdENU(), plan_yaw_, plan_acceleration_);
            acc_yawrate_cmd_publish(vel_controller_.getAttitudeCmdAcc(),vel_controller_.getYawrateCmd());
        } 
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
    local_pub_.publish(msg);
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
    msg.type_mask = IGNORE_ROLL_RATE + IGNORE_PITCH_RATE + IGNORE_YAW_RATE_ATT;  // Ignore rate messages
    msg.orientation.w = quatcmd[0];
    msg.orientation.x = quatcmd[1];
    msg.orientation.y = quatcmd[2];
    msg.orientation.z = quatcmd[3];
    msg.thrust = thrcmd;
    att_pub_.publish(msg);
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
OffboardSO3Node::acc_yawrate_cmd_publish(const Eigen::Vector3d &acccmd, const double &yawratecmd)
{
    mavros_msgs::PositionTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "acc_yawrate";
    msg.coordinate_frame = FRAME_LOCAL_NED;
    msg.type_mask = IGNORE_PX + IGNORE_PY + IGNORE_PZ + IGNORE_VX + IGNORE_VY \
                    + IGNORE_VZ + IGNORE_YAW;  //ignore pos vel yaw messages, don't use force
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 0;
    msg.velocity.x = 0;
    msg.velocity.y = 0;
    msg.velocity.z = 0;
    msg.acceleration_or_force.x = acccmd[0];
    msg.acceleration_or_force.y = acccmd[1];
    msg.acceleration_or_force.z = acccmd[2];
    msg.yaw = 0;
    msg.yaw_rate = yawratecmd;
    local_pub_.publish(msg);
    if (att_cnt % 50 == 0)
    {
        ROS_INFO("acc and yawrate cmd : %f %f %f %f", \
        msg.acceleration_or_force.x, msg.acceleration_or_force.y, msg.acceleration_or_force.z, msg.yaw_rate);
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
    // while(ros::ok() && !current_state_.connected)
    // {
    //     ros::spinOnce();
    //     ROS_INFO("Connecting...");
        
    // }
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

void
OffboardSO3Node::dynamicReconfigureCallback(offboard_so3::OffboardSo3ControllerConfig &config,
                                                 uint32_t level)
{
    if (my_TWratio != config.tw_ratio) 
    {
        my_TWratio = config.tw_ratio;
        ROS_INFO("Reconfigure request : TWratio = %.2f ", my_TWratio);
    } 
    // pos_kp
    if (my_pos_kp[0] != config.pos_kp_x) 
    {
        my_pos_kp[0] = config.pos_kp_x;
        ROS_INFO("Reconfigure request : pos_kp_x  = %.2f  ", my_pos_kp[0]);
    }
    if (my_pos_kp[1] != config.pos_kp_y) 
    {
        my_pos_kp[1] = config.pos_kp_y;
        ROS_INFO("Reconfigure request : pos_kp_y  = %.2f  ", my_pos_kp[1]);
    } 
    if (my_pos_kp[2] != config.pos_kp_z) 
    {
        my_pos_kp[2] = config.pos_kp_z;
        ROS_INFO("Reconfigure request : pos_kp_z  = %.2f  ", my_pos_kp[2]);
    } 
    if (my_pos_kp[3] != config.pos_kp_yaw) 
    {
        my_pos_kp[3] = config.pos_kp_yaw;
        ROS_INFO("Reconfigure request : pos_kp_yaw  = %.2f  ", my_pos_kp[3]);
    } 
    // vel_kp
    if (my_vel_kp[0] != config.vel_kp_x) 
    {
        my_vel_kp[0] = config.vel_kp_x;
        ROS_INFO("Reconfigure request : vel_kp_x =%.2f  ", my_vel_kp[0]);
    } 
    if (my_vel_kp[1] != config.vel_kp_y) 
    {
        my_vel_kp[1] = config.vel_kp_y;
        ROS_INFO("Reconfigure request : vel_kp_y  = %.2f  ", my_vel_kp[1]);
    }
    if (my_vel_kp[2] != config.vel_kp_z) 
    {
        my_vel_kp[2] = config.vel_kp_z;
        ROS_INFO("Reconfigure request : vel_kp_z  = %.2f  ", my_vel_kp[2]);
    }
    if (my_vel_kp[3] != config.vel_kp_yaw) 
    {
        my_vel_kp[3] = config.vel_kp_yaw;
        ROS_INFO("Reconfigure request : vel_kp_yaw  = %.2f  ", my_vel_kp[3]);
    }
    //vel_ki
    if (my_vel_ki[0] != config.vel_ki_x) 
    {
        my_vel_ki[0] = config.vel_ki_x;
        ROS_INFO("Reconfigure request : vel_ki_x  = %.2f  ", my_vel_ki[0]);
    }
    if (my_vel_ki[1] != config.vel_ki_y) 
    {
        my_vel_ki[1] = config.vel_ki_y;
        ROS_INFO("Reconfigure request : vel_ki_y  = %.2f  ", my_vel_ki[1]);
    }
    if (my_vel_ki[2] != config.vel_ki_z) 
    {
        my_vel_ki[2] = config.vel_ki_z;
        ROS_INFO("Reconfigure request : vel_ki_z  = %.2f  ", my_vel_ki[2]);
    }
    if (my_vel_ki[3] != config.vel_ki_yaw) 
    {
        my_vel_ki[3] = config.vel_ki_yaw;
        ROS_INFO("Reconfigure request : vel_ki_yaw  = %.2f  ", my_vel_ki[3]);
    }
    //vel_i_satur
    if (my_vel_xyz_i_errsatur[0] != config.vel_i_satur_x) 
    {
        my_vel_xyz_i_errsatur[0] = config.vel_i_satur_x;
        ROS_INFO("Reconfigure request : vel_i_satur_x  = %.2f  ", my_vel_xyz_i_errsatur[0]);
    }
    if (my_vel_xyz_i_errsatur[1] != config.vel_i_satur_y) 
    {
        my_vel_xyz_i_errsatur[1] = config.vel_i_satur_y;
        ROS_INFO("Reconfigure request : vel_i_satur_y  = %.2f  ", my_vel_xyz_i_errsatur[1]);
    }
    if (my_vel_xyz_i_errsatur[2] != config.vel_i_satur_z) 
    {
        my_vel_xyz_i_errsatur[2] = config.vel_i_satur_z;
        ROS_INFO("Reconfigure request : vel_i_satur_z  = %.2f  ", my_vel_xyz_i_errsatur[2]);
    }
    if (my_vel_yawrate_i_errsatur != config.vel_i_satur_yaw) 
    {
        my_vel_yawrate_i_errsatur = config.vel_i_satur_yaw;
        ROS_INFO("Reconfigure request : vel_i_satur_yaw  = %.2f  ", my_vel_yawrate_i_errsatur);
    }
    //plan
    if (my_plan_pos[0] != config.plan_pos_x) 
    {
        my_plan_pos[0] = config.plan_pos_x;
        ROS_INFO("Reconfigure request : plan_pos_x  = %.2f  ", my_plan_pos[0]);
    }
    if (my_plan_pos[1] != config.plan_pos_y) 
    {
        my_plan_pos[1] = config.plan_pos_y;
        ROS_INFO("Reconfigure request : plan_pos_x  = %.2f  ", my_plan_pos[1]);
    }
    if (my_plan_pos[2] != config.plan_pos_z) 
    {
        my_plan_pos[2] = config.plan_pos_z;
        ROS_INFO("Reconfigure request : plan_pos_x  = %.2f  ", my_plan_pos[2]);
    }
        if (my_plan_pos[0] != config.plan_pos_x) 
    {
        my_plan_pos[0] = config.plan_pos_x;
        ROS_INFO("Reconfigure request : plan_pos_x  = %.2f  ", my_plan_pos[0]);
    }
    if (my_plan_pos[1] != config.plan_pos_y) 
    {
        my_plan_pos[1] = config.plan_pos_y;
        ROS_INFO("Reconfigure request : plan_pos_y  = %.2f  ", my_plan_pos[1]);
    }
    if (my_plan_pos[2] != config.plan_pos_z) 
    {
        my_plan_pos[2] = config.plan_pos_z;
        ROS_INFO("Reconfigure request : plan_pos_z  = %.2f  ", my_plan_pos[2]);
    }
    if (my_plan_vel[0] != config.plan_vel_x) 
    {
        my_plan_vel[0] = config.plan_vel_x;
        ROS_INFO("Reconfigure request : plan_vel_x  = %.2f  ", my_plan_vel[0]);
    }
    if (my_plan_vel[1] != config.plan_vel_y) 
    {
        my_plan_vel[1] = config.plan_vel_y;
        ROS_INFO("Reconfigure request : plan_vel_y  = %.2f  ", my_plan_vel[1]);
    }
    if (my_plan_vel[2] != config.plan_vel_z) 
    {
        my_plan_vel[2] = config.plan_vel_z;
        ROS_INFO("Reconfigure request : plan_vel_z  = %.2f  ", my_plan_vel[2]);
    }
    if (my_plan_yaw != config.plan_yaw) 
    {
        my_plan_yaw = config.plan_yaw;
        ROS_INFO("Reconfigure request : plan_yaw  = %.2f  ", my_plan_yaw);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_so3_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    int musk = POS_YAWRATE_NED;
    OffboardSO3Node OffboardSO3Controller(nh, nh_private, musk);

    dynamic_reconfigure::Server<offboard_so3::OffboardSo3ControllerConfig> srv;
    dynamic_reconfigure::Server<offboard_so3::OffboardSo3ControllerConfig>::CallbackType f;
    f = boost::bind(&OffboardSO3Node::dynamicReconfigureCallback, OffboardSO3Controller, _1, _2);
    srv.setCallback(f);

    ros::spin();
    return 0;
}