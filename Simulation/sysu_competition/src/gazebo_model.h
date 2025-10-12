#include "ros_msg_utils.h"

#define FLIP_ANGLE M_PI/3
#define RC_DEADZONE 0.2

class Gazebo_model
{
private:
    std::string node_name;  // 节点名称
    int uav_id;             // 【参数】无人机ID
    std::string uav_name;   // 【参数】无人机名称

    // 无人机飞行相关参数
    struct Gazebomodel
    {
        string model_name;
        Eigen::Vector3d init_pos{0.0, 0.0, 0.0};     // 降落点
        double init_yaw{0.0};
        Eigen::Vector3d model_pos{0.0, 0.0, 0.0};     // 降落点
        double model_yaw{0.0}; 
    };
    Gazebomodel uav1;
    Gazebomodel uav2;
    Gazebomodel uav3;

    // 订阅句柄
    ros::Subscriber uav1_gazebo_pose_sub;
    ros::Subscriber uav2_gazebo_pose_sub;
    ros::Subscriber uav3_gazebo_pose_sub;
    void uav1_gazebo_pose_cb(const nav_msgs::Odometry::ConstPtr &msg);  
    void uav2_gazebo_pose_cb(const nav_msgs::Odometry::ConstPtr &msg);  
    void uav3_gazebo_pose_cb(const nav_msgs::Odometry::ConstPtr &msg);  



    ros::Publisher model_state_pub;    // 【订阅】控制指令订阅


public:
    Gazebo_model() {};
    ~Gazebo_model() {};

    void init(ros::NodeHandle &nh); 
    void debug();

    void set_model_state(string model_name, double x, double y, double z, double yaw);


};