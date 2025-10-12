#include "gazebo_model.h"

// 中断信号
void mySigintHandler(int sig)
{
    ROS_INFO("[gazebo_model_node] exit...");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
 
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "gazebo_model_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    // 中断信号注册 
    signal(SIGINT, mySigintHandler);

    // 读取参数
    bool flag_printf; // 是否打印状态
    nh.param<bool>("flag_printf", flag_printf, true);           

    // 初始化外部估计类
    Gazebo_model test;
    test.init(nh);

    ros::Time last_time = ros::Time::now();
    // 主循环
    while (ros::ok)
    {
        ros::spinOnce();

        // 定时打印状态
        if (ros::Time::now() - last_time > ros::Duration(1.0) && flag_printf)
        {
            test.debug();
            test.set_model_state("uav_1", 10,7.0,5,0.0/180.0*M_PI);
            test.set_model_state("uav_2", 10,9.0,5,90.0/180.0*M_PI);
            test.set_model_state("uav_3", 10,8.0,5,180.0/180.0*M_PI);
            last_time = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}