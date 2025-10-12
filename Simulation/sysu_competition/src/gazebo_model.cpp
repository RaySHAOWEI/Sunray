#include "gazebo_model.h"

void Gazebo_model::init(ros::NodeHandle &nh)
{
    node_name = ros::this_node::getName();
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名字前缀
    // 无人机名字 = 无人机名字前缀 + 无人机ID
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 【订阅】Gazebo pose
    uav1_gazebo_pose_sub = nh.subscribe<nav_msgs::Odometry>("/uav_1/gazebo_pose", 10, &Gazebo_model::uav1_gazebo_pose_cb, this);
    uav2_gazebo_pose_sub = nh.subscribe<nav_msgs::Odometry>("/uav_2/gazebo_pose", 10, &Gazebo_model::uav2_gazebo_pose_cb, this);
    uav3_gazebo_pose_sub = nh.subscribe<nav_msgs::Odometry>("/uav_3/gazebo_pose", 10, &Gazebo_model::uav3_gazebo_pose_cb, this);

    // 【发布】设置模型位置
    model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);




}

void Gazebo_model::set_model_state(string model_name, double x, double y, double z, double yaw)
{
    gazebo_msgs::ModelState model_state;

    model_state.model_name = model_name;
    model_state.pose.position.x = x;
    model_state.pose.position.y = y;
    model_state.pose.position.z = z;

    Eigen::Vector3d att;
    att << 0.0, 0.0, yaw;
    Eigen::Quaterniond q_des = quaternion_from_rpy(att);
    model_state.pose.orientation.x = q_des.x();
    model_state.pose.orientation.y = q_des.y();
    model_state.pose.orientation.z = q_des.z();
    model_state.pose.orientation.w = q_des.w();
    model_state.reference_frame = "ground_plane::link";    
    model_state_pub.publish(model_state);
}


void Gazebo_model::uav1_gazebo_pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav1.model_pos[0] = msg->pose.pose.position.x;
    uav1.model_pos[1] = msg->pose.pose.position.y;
    uav1.model_pos[2] = msg->pose.pose.position.z;

    // 转为rpy
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    uav1.model_yaw = yaw;
}
void Gazebo_model::uav2_gazebo_pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav2.model_pos[0] = msg->pose.pose.position.x;
    uav2.model_pos[1] = msg->pose.pose.position.y;
    uav2.model_pos[2] = msg->pose.pose.position.z;

    // 转为rpy
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    uav2.model_yaw = yaw;
}
void Gazebo_model::uav3_gazebo_pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav3.model_pos[0] = msg->pose.pose.position.x;
    uav3.model_pos[1] = msg->pose.pose.position.y;
    uav3.model_pos[2] = msg->pose.pose.position.z;

    // 转为rpy
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    uav3.model_yaw = yaw;
}


void Gazebo_model::debug()
{
    Logger::print_color(int(LogColor::green), "1号无人机位置[X Y Z yaw]:",
                        uav1.model_pos[0],
                        uav1.model_pos[1],
                        uav1.model_pos[2],
                        "[ m ]",
                        uav1.model_yaw / M_PI * 180,
                        "[deg]");


    Logger::print_color(int(LogColor::green), "2号无人机位置[X Y Z yaw]:",
                        uav2.model_pos[0],
                        uav2.model_pos[1],
                        uav2.model_pos[2],
                        "[ m ]",
                        uav2.model_yaw / M_PI * 180,
                        "[deg]");

    Logger::print_color(int(LogColor::green), "3号无人机位置[X Y Z yaw]:",
                        uav3.model_pos[0],
                        uav3.model_pos[1],
                        uav3.model_pos[2],
                        "[ m ]",
                        uav3.model_yaw / M_PI * 180,
                        "[deg]");
}

