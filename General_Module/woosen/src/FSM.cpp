#include "FSM.h"

FSM::FSM() : 
    current_mode_(DISABLE),
    flight_state_(AUTO_DISABLE_FLIGHT),
    rolling_state_(AUTO_DISABLE_ROLLING),
    rate_(60.0) {
    // 初始化通道数据
    for (int i = 0; i < CHANNEL_LENGTH; i++) {
        channel_[i] = 0;
        last_channel_[i] = 0;
    }
}

FSM::~FSM() {
    // 析构函数
}

bool FSM::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    // 订阅 /mavros/rc/in 话题
    rc_sub_ = nh_.subscribe("/mavros/rc/in", 10, &FSM::rcCallback, this);
    
    ROS_INFO("FSM Node Initialized. Waiting for RC data...");
    
    return true;
}

void FSM::run() {
    while (ros::ok()) {
        // 更新状态机逻辑
        updateState();
        
        // 更新上一次的通道数据
        for (int i = 0; i < CHANNEL_LENGTH; i++) {
            last_channel_[i] = channel_[i];
        }
        
        ros::spinOnce();
        rate_.sleep();
    }
}

void FSM::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    for (int i = 0; i < CHANNEL_LENGTH; i++) {
        channel_[i] = msg->channels[i];
    }
}

void FSM::updateState() {
    // 更新飞行状态（通道7）
    if (channel_[7] < 1250) {
        flight_state_ = AUTO_DISABLE_FLIGHT;
        // 手动接管，只控制油门量
    } else if (channel_[7] > 1750) {
        flight_state_ = AUTO_ENABLE_FLIGHT;
        // 自动开启，运行自动避障等程序
    } else {
        // 可能没收到信号
    }
    
    // 更新模式（通道9）
    if (channel_[9] < 1200) {
        current_mode_ = DISABLE;
        // 手动模式，程序不干预
    } else if (channel_[9] < 1700 && channel_[9] > 1300) {
        current_mode_ = ROLLING;
        // 滚动模式
    } else if (channel_[9] > 1800) {
        current_mode_ = FLIGHT;
        // 飞行模式
    }
}

FSM::ModeSwitch FSM::getCurrentMode() const {
    return current_mode_;
}

FSM::FlightState FSM::getFlightState() const {
    return flight_state_;
}

FSM::RollingState FSM::getRollingState() const {
    return rolling_state_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "FSM_node");
    ros::NodeHandle nh;
    
    // 创建FSM实例
    FSM fsm;
    
    // 初始化FSM
    if (!fsm.init(nh)) {
        ROS_ERROR("Failed to initialize FSM.");
        return -1;
    }
    
    // 运行FSM
    fsm.run();
    
    return 0;
}