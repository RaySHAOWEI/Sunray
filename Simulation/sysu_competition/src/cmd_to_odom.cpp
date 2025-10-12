#include "cmd_to_odom.h"

namespace cmd_to_odom_ns
{

void cmd_to_odom::init(ros::NodeHandle& nh, int id)
{
    nh.param("fake_odom/quad_mass", quad_mass, 1.0f);
    nh.param("fake_odom/gravity", gravity, 9.8f);
    nh.param("fake_odom/dt", dt, 0.01);
    nh.param("fake_odom/tilt_angle_max", tilt_angle_max, 10.0f);
    nh.param("fake_odom/k_pos", k_pos, 0.5f);
    nh.param("fake_odom/k_vel", k_vel, 1.0f);

    uav_id = id;
    uav_name = "/uav" + std::to_string(uav_id);
    model_name = "cxy_fake_solo_" + std::to_string(uav_id);
    get_pos_cmd = false;
    get_att_cmd = false;
    get_ego_cmd = false;
    get_fake_odom = false;

    pos_cmd_sub = nh.subscribe<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 1, &cmd_to_odom::pos_cmd_cb, this);
    // att_cmd_sub   = nh.subscribe<mavros_msgs::AttitudeTarget>(uav_name +  "/mavros/setpoint_raw/attitude", 1, &cmd_to_odom::att_cmd_cb, this);
    ego_cmd_sub  = nh.subscribe<quadrotor_msgs::PositionCommand>( uav_name + "/prometheus/ego/traj_cmd", 1, &cmd_to_odom::ego_cmd_cb, this);
    odom_pub       = nh.advertise<nav_msgs::Odometry>(uav_name + "/prometheus/fake_odom", 1);
    
    fake_odom_timer = nh.createTimer(ros::Duration(dt), &cmd_to_odom::fake_odom_cb, this);
    pub_timer = nh.createTimer(ros::Duration(0.02), &cmd_to_odom::pub_cb, this);
    // debug_timer = nh.createTimer(ros::Duration(0.1), &cmd_to_odom::debug_cb, this);

    // 初始化模型位置
    init_pos << 0.0,0.0,0.0;
    init_yaw = 0.0;
    fake_pos = init_pos;
    fake_vel << 0.0,0.0,0.0;
    fake_acc << 0.0,0.0,0.0;
    fake_euler << 0.0,0.0,init_yaw;
    fake_quat2 = quaternion_from_rpy(fake_euler);
    fake_quat.x = fake_quat2.x();
    fake_quat.y = fake_quat2.y();
    fake_quat.z = fake_quat2.z();
    fake_quat.w = fake_quat2.w();
    fake_Rb = fake_quat2.toRotationMatrix();
    // 变量初始化
    pos_sp << 0.0 , 0.0 , 0.0;
    vel_sp << 0.0 , 0.0 , 0.0;
    acc_sp << 0.0 , 0.0 , 0.0;
    euler_sp << 0.0 , 0.0 , 0.0;
    thrust_body_sp << 0.0 , 0.0 , 0.0;
    thrust_enu_sp << 0.0 , 0.0 , 0.0;
    quat2_sp = quaternion_from_rpy(euler_sp);
    Rb_sp = quat2_sp.toRotationMatrix();
    throttle_sp = 0.0;
}

void cmd_to_odom::pos_cmd_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    get_pos_cmd = true;
    if(msg->type_mask == 0x4000)
    {
        pos_sp << 0.0 , 0.0 , 0.0;
        vel_sp << 0.0 , 0.0 , 0.0;
        euler_sp << 0.0 , 0.0 , 0.0;
        throttle_sp = 0.0;
        fake_thrust << 0.0 , 0.0 , quad_mass * gravity;
        pos_control_type = POS_TYPE::IDLE;
        get_pos_cmd = false;
    }else if(msg->type_mask == 0b100111111000)
    {
        // send_pos_setpoint + yaw
        pos_sp << msg->position.x , msg->position.y , msg->position.z;
        vel_sp << 0.0 , 0.0 , 0.0;
        euler_sp(2) = msg->yaw;
        pos_control_type = POS_TYPE::XYZ_POS;
    }else if(msg->type_mask == 0b100111000111)
    {
        // send_vel_setpoint + yaw
        pos_sp << 0.0 , 0.0 , 0.0;
        vel_sp << msg->velocity.x , msg->velocity.y , msg->velocity.z;
        euler_sp(2) = msg->yaw;
        pos_control_type = POS_TYPE::XYZ_VEL;
    }else if(msg->type_mask == 0b100111000011)
    {
        // send_vel_xy_pos_z_setpoint + yaw
        pos_sp << 0.0 , 0.0 , msg->position.z;
        vel_sp << msg->velocity.x , msg->velocity.y , 0.0;
        euler_sp(2) = msg->yaw;
        pos_control_type = POS_TYPE::XY_VEL_Z_POS;
    }else if(msg->type_mask == 0b100111000000)
    {
        // send_pos_vel_xyz_setpoint + yaw
        pos_sp << msg->position.x , msg->position.y , msg->position.z;
        vel_sp << msg->velocity.x , msg->velocity.y , msg->velocity.z;
        euler_sp(2) = msg->yaw;
        pos_control_type = POS_TYPE::XYZ_POS_VEL;
    }else
    {
        get_pos_cmd = false;
        cout << RED << "wrong pos_cmd type_mask."<< TAIL << endl;
    }

    // 模拟位置控制，得到
    fake_pos_control();
}

// 输入：fake_pos，fake_vel
// 输出：thrust_enu_sp，fake_Rb
void cmd_to_odom::fake_pos_control()
{
    if(pos_control_type == POS_TYPE::IDLE)
    {
        euler_sp(0)  = 0.0;
        euler_sp(1)  = 0.0;
        thrust_enu_sp << 0.0 , 0.0, 0.0;
        return;
    }

    Eigen::Vector3d pos_error,vel_error;
    Eigen::Vector3d u_cmd;
    if(pos_control_type ==  POS_TYPE::XYZ_POS)
    {
        pos_error = pos_sp - fake_pos;
        vel_error  = k_pos * pos_error - fake_vel;
        u_cmd = k_vel * vel_error;
    }else if(pos_control_type ==  POS_TYPE::XYZ_VEL)
    {
        vel_error  = vel_sp - fake_vel;
        u_cmd = k_vel * vel_error;
    }else if(pos_control_type ==  POS_TYPE::XY_VEL_Z_POS)
    {
        pos_error(2) = pos_sp(2) - fake_pos(2);
        vel_error(0)  = vel_sp(0) - fake_vel(0);
        vel_error(1)  = vel_sp(1) - fake_vel(1);
        vel_error(2)  = k_pos * pos_error(2) - fake_vel(2);
        u_cmd = k_vel * vel_error;
    }else if(pos_control_type ==  POS_TYPE::XYZ_POS_VEL)
    {
        // 目前以位置为准（针对Land）
        pos_error = pos_sp - fake_pos;
        vel_error  = k_pos * pos_error - fake_vel;
        u_cmd = k_vel * vel_error;
    } 

	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
    Eigen::Vector3d F_des;
    u_cmd(2) = u_cmd(2) + gravity;
	F_des = u_cmd * quad_mass;
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * quad_mass * gravity)
	{
		F_des = F_des / F_des(2) * (0.5 * quad_mass * gravity);
	}
	else if (F_des(2) > 2 * quad_mass * gravity)
	{
		F_des = F_des / F_des(2) * (2 * quad_mass * gravity);
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(uav_utils::toRad(tilt_angle_max)))
	{
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(uav_utils::toRad(tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(uav_utils::toRad(tilt_angle_max)))
	{
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(uav_utils::toRad(tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = uav_utils::rotz(fake_euler(2));
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    euler_sp(0)  = std::atan2(-fy, fz);
    euler_sp(1)  = std::atan2( fx, fz);
    // 缺偏航角动态（位置控制认为直接赋值，euler_sp(2)已经在前面赋值过了）
    //euler_sp(2)  = euler_sp(2);
    
    thrust_enu_sp = F_des;
}

void cmd_to_odom::att_cmd_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    if(msg->type_mask == 0b00000111 )
    {
        get_att_cmd = true;
        // 放弃
        // 期望姿态
        // quat_sp = msg->orientation;
        // quat2_sp.w() = msg->orientation.w;
        // quat2_sp.x() = msg->orientation.x;
        // quat2_sp.y() = msg->orientation.y;
        // quat2_sp.z() = msg->orientation.z;    
        // Rb_sp = quat2_sp.toRotationMatrix();
        // euler_sp = quaternion_to_euler(quat2_sp);
        fake_quat = msg->orientation;
        fake_quat2.w() = msg->orientation.w;
        fake_quat2.x() = msg->orientation.x;
        fake_quat2.y() = msg->orientation.y;
        fake_quat2.z() = msg->orientation.z;   
        fake_Rb = fake_quat2.toRotationMatrix();
        fake_euler = quaternion_to_euler(fake_quat2);

        // // 期望油门
        throttle_sp = msg->thrust;
        // // 根据姿态和油门计算期望三轴推力（合力，不考虑重力）
        // // 此处att_control计算得到的控制指令 是没有考虑速度的
        thrust_body_sp << 0.0, 0.0, quad_mass*gravity* throttle_sp / hover_per;
        thrust_enu_sp = fake_Rb *  thrust_body_sp;
    }else
    {
        get_att_cmd = false;
        cout << RED << "wrong att_cmd type_mask."<< TAIL << endl;
        return;
    }
}

void cmd_to_odom::ego_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    if(msg->velocity.x == 0 && msg->velocity.y == 0)
    {
        get_ego_cmd = false;
    }else
    {
        get_pos_cmd = false;
        get_ego_cmd = true;
        get_fake_odom = true;

        fake_pos(0) = msg->position.x;
        fake_pos(1) = msg->position.y;
        fake_pos(2) = msg->position.z;
        fake_vel(0) = msg->velocity.x;
        fake_vel(1) = msg->velocity.y;
        fake_vel(2) = msg->velocity.z;
        fake_acc(0) = msg->acceleration.x;
        fake_acc(1) = msg->acceleration.y;
        fake_acc(2) = msg->acceleration.z;
        // 根据加速度计算姿态角
        Eigen::Vector3d alpha = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
        Eigen::Vector3d xC(cos(msg->yaw), sin(msg->yaw), 0);
        Eigen::Vector3d yC(-sin(msg->yaw), cos(msg->yaw), 0);
        Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
        Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
        Eigen::Vector3d zB = xB.cross(yB);
        fake_Rb.col(0) = xB;
        fake_Rb.col(1) = yB;
        fake_Rb.col(2) = zB;
        Eigen::Quaterniond q(fake_Rb);
        fake_quat2 = q;
        fake_euler = quaternion_to_euler(fake_quat2);
        fake_quat.x = fake_quat2.x();
        fake_quat.y = fake_quat2.y();
        fake_quat.z = fake_quat2.z();
        fake_quat.w = fake_quat2.w();
    }
}

// 输入：thrust_enu_sp，euler_sp, 上一时刻的位姿
// 输出：fake_odom
void cmd_to_odom::fake_odom_cb(const ros::TimerEvent &e)
{
    if(!get_pos_cmd)
    {
        return;
    }

    // 使用EGO cmd时 不需要模拟位置
    if(get_ego_cmd)
    {
        return;
    }

    // 计算期望推力
    fake_thrust = thrust_enu_sp;
    fake_thrust(2) = fake_thrust(2) - quad_mass*gravity;
    // 计算加速度,速度,位置
    fake_acc = fake_thrust /quad_mass;
    fake_vel = fake_vel + fake_acc * dt;
    fake_pos = fake_pos + fake_vel * dt;
    // 计算姿态角(无角度动态,可以考虑加入一个一阶滤波)
    fake_euler = euler_sp;
    fake_quat2 = quaternion_from_rpy(fake_euler);
    fake_quat.x = fake_quat2.x();
    fake_quat.y = fake_quat2.y();
    fake_quat.z = fake_quat2.z();
    fake_quat.w = fake_quat2.w();
    fake_Rb = fake_quat2.toRotationMatrix();

    get_fake_odom = true;
}

Eigen::Vector3d cmd_to_odom::get_uav_pos()
{
    return fake_pos;
}

gazebo_msgs::ModelState cmd_to_odom::get_model_state()
{
    // gazebo_state.model_name = model_name;
    // gazebo_state.pose.position.x = fake_pos[0];
    // gazebo_state.pose.position.y = fake_pos[1];
    // gazebo_state.pose.position.z = fake_pos[2];
    // gazebo_state.pose.orientation = fake_quat;
    // gazebo_state.reference_frame = "ground_plane::link";

    return gazebo_state;
}

void cmd_to_odom::set_init_pos(Eigen::Vector3d pos, double yaw)
{
    init_pos = pos;
    init_yaw = yaw;
    fake_pos = init_pos;
    fake_euler(2) = init_yaw;
    cout << GREEN  << "Set cxy_fake_solo_" << uav_id << " init position:  "<< init_pos[0]<<" [ m ] "<<init_pos[1]<<" [ m ] "<< TAIL <<endl;
}

void cmd_to_odom::pub_cb(const ros::TimerEvent &e)
{
    if(!get_fake_odom)
    {
        // 发布初始位置
        gazebo_state.model_name = model_name;
        gazebo_state.pose.position.x = init_pos[0];
        gazebo_state.pose.position.y = init_pos[1];
        gazebo_state.pose.position.z = init_pos[2];
        fake_euler << 0.0,0.0,init_yaw;
        fake_quat2 = quaternion_from_rpy(fake_euler);
        gazebo_state.pose.orientation.x = fake_quat2.x();
        gazebo_state.pose.orientation.y = fake_quat2.y();
        gazebo_state.pose.orientation.z = fake_quat2.z();
        gazebo_state.pose.orientation.w = fake_quat2.w();
        gazebo_state.reference_frame = "ground_plane::link";
        return;
    }

    // 发布fake odom
    fake_odom.header.stamp = ros::Time::now();
    fake_odom.header.frame_id = "world";
    fake_odom.child_frame_id = "base_link";
    fake_odom.pose.pose.position.x = fake_pos[0];
    fake_odom.pose.pose.position.y = fake_pos[1];
    fake_odom.pose.pose.position.z = fake_pos[2];
    fake_odom.pose.pose.orientation = fake_quat;
    fake_odom.twist.twist.linear.x = fake_vel[0];
    fake_odom.twist.twist.linear.y = fake_vel[1];
    fake_odom.twist.twist.linear.z = fake_vel[2];
    odom_pub.publish(fake_odom);

    // 发布 gazebo model state
    // 注意：这个话题发布的是相对位置，且无法移除初始位置的影响（因此将所有无人机初始位置设为0）
    // 注意：他这个坐标转换是先转角度再加位移
    gazebo_state.model_name = model_name;
    gazebo_state.pose.position.x = fake_pos[0];
    gazebo_state.pose.position.y = fake_pos[1];
    gazebo_state.pose.position.z = fake_pos[2];
    gazebo_state.pose.orientation = fake_quat;
    gazebo_state.reference_frame = "ground_plane::link";
}

void cmd_to_odom::debug_cb(const ros::TimerEvent &e)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Fake Odom <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN  << "UAV_id : " <<  uav_id << "   UAV_name : " <<  uav_name << TAIL << endl;
    cout << GREEN  << "pos_control_type : " <<  pos_control_type  << TAIL << endl;
    cout << GREEN  << "pos_sp [X Y Z]         : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<<pos_sp[2]<<" [ m ] "<< TAIL <<endl;
    cout << GREEN <<  "vel_sp  [X Y Z]         : " << vel_sp[0] << " [m/s] "<< vel_sp[1]<<" [m/s] "<<vel_sp[2]<<" [m/s] "<< TAIL <<endl;
    cout << GREEN <<  "euler_sp                   : " << euler_sp[0] << " [rad] "<< euler_sp[1]<<" [rad] "<<euler_sp[2]<<" [rad] "<< TAIL <<endl;
    cout << GREEN <<  "throttle_sp[0-1]    : " << throttle_sp<< TAIL <<endl;
    cout << GREEN  << "fake_pos [X Y Z]    : " << fake_pos[0] << " [ m ] "<< fake_pos[1]<<" [ m ] "<<fake_pos[2]<<" [ m ] "<< TAIL <<endl;
    cout << GREEN <<  "fake_vel  [X Y Z]    : " << fake_vel[0] << " [m/s] "<< fake_vel[1]<<" [m/s] "<<fake_vel[2]<<" [m/s] "<< TAIL <<endl;
    cout << GREEN <<  "fake_euler              : " << fake_euler[0] << " [rad] "<< fake_euler[1]<<" [rad] "<<fake_euler[2]<<" [rad] "<< TAIL <<endl;
}

}