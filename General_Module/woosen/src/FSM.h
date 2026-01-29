#ifndef FSM_H
#define FSM_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include "ros_msg_utils.h"

class FSM {
public:
    static constexpr int CHANNEL_LENGTH = 10;    // 10通道 遥控上 数值减 数值在1000 - 2000 之间，中值为1500

    typedef enum {
        DISABLE,
        ROLLING,
        FLIGHT,
    } ModeSwitch;

    typedef enum {
        AUTO_DISABLE,
        AUTO_ENABLE
    } Auto_state;

    typedef enum {
        FUNC_DISABLE,       // 使用px4内置POSCTL模式
        ROLLING_MANUAL,     // 将油门设定为60%，其他同DISABLE
        ROLLING_AUTO,       // 将油门设定为60%，并且通过深度图识别到前方障碍物时，自动起飞避障，前方无障碍物时自动回地面，其他同ROLLING_MANUAL
        FLIGHT_MANUAL,      // 进入这个状态时，执行一次将高度设定为0.8m的指令，后续同DISABLE
        FLIGHT_AUTO,        // 将高度设定为0.8m，调用深度图进行简易飞行避障
        COUNT               // 用于计数
    } FUNCTION_STATE;

    FSM();
    ~FSM();

    // 初始化FSM
    bool init(ros::NodeHandle& nh);

    // 运行FSM主循环
    void run();

    // 获取当前模式
    ModeSwitch getCurrentMode() const;
    Auto_state getAutoState() const;
    FUNCTION_STATE getFunctionState() const;
    FUNCTION_STATE getLastFunctionState() const;

    // 查看无人机是否解锁
    bool isUAVArmed() const {
        return uav_state_.armed;
    }

private:
    // 无人机状态回调函数
    void uavStateCallback(const sunray_msgs::UAVState::ConstPtr &msg);

    // RC通道回调函数
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

    // 更新状态机逻辑
    void updateState();

    // 具体状态处理函数
    void handleDisable();
    void handleRollingManual();
    void handleRollingAuto();
    void handleFlightManual();
    void handleFlightAuto();

    // 具体状态处理函数指针数组
    static constexpr size_t STATE_COUNT = static_cast<size_t>(FUNCTION_STATE::COUNT);
    static void (FSM::*stateHandlers_[STATE_COUNT])();

    // ROS节点句柄
    ros::NodeHandle nh_;
    std::string node_name_;
    std::string state_name_;

    // 订阅者
    ros::Subscriber uav_state_sub_;
    ros::Subscriber rc_sub_;

    // 发布者
    ros::Publisher control_cmd_pub_;
    ros::Publisher uav_setup_pub_;
    ros::Publisher setpoint_pub_;
    
    // 无人机状态和控制
    sunray_msgs::UAVState uav_state_;
    sunray_msgs::UAVControlCMD uav_cmd_;
    sunray_msgs::UAVSetup uav_setup_;

    // RC通道数据
    int channel_[CHANNEL_LENGTH];
    int last_channel_[CHANNEL_LENGTH];

    // 当前状态
    ModeSwitch current_mode_;
    Auto_state auto_state_;
    FUNCTION_STATE function_state_;
    FUNCTION_STATE last_function_state_;

    // 运行速率
    ros::Rate rate_;
};

#endif // FSM_H