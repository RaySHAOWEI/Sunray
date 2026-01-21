#ifndef FSM_H
#define FSM_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

class FSM {
public:
    static constexpr int CHANNEL_LENGTH = 10;    // 10通道 遥控上 数值减 数值在1000 - 2000 之间，中值为1500

    typedef enum {
        DISABLE,
        ROLLING,
        FLIGHT,
    } ModeSwitch;

    typedef enum {
        AUTO_DISABLE_FLIGHT,
        AUTO_ENABLE_FLIGHT
    } FlightState;

    typedef enum {
        AUTO_DISABLE_ROLLING,
        AUTO_ENABLE_ROLLING
    } RollingState;

    FSM();
    ~FSM();

    // 初始化FSM
    bool init(ros::NodeHandle& nh);

    // 运行FSM主循环
    void run();

    // 获取当前模式
    ModeSwitch getCurrentMode() const;
    FlightState getFlightState() const;
    RollingState getRollingState() const;

private:
    // RC通道回调函数
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

    // 更新状态机逻辑
    void updateState();

    // ROS节点句柄
    ros::NodeHandle nh_;

    // RC通道订阅者
    ros::Subscriber rc_sub_;

    // RC通道数据
    int channel_[CHANNEL_LENGTH];
    int last_channel_[CHANNEL_LENGTH];

    // 当前状态
    ModeSwitch current_mode_;
    FlightState flight_state_;
    RollingState rolling_state_;

    // 运行速率
    ros::Rate rate_;
};

#endif // FSM_H