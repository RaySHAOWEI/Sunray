#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>

// 两种状态
enum State {
    DISABLE,
    ROLLING_AUTO
};

class RollingController {
public:
    RollingController() : nh_() {
        // 订阅
        rc_sub_ = nh_.subscribe("/uav1/mavros/rc/in", 10, &RollingController::rcCallback, this);
        
        // 订阅深度图像
        depth_sub_ = nh_.subscribe(depth_image_topic_, 1, &RollingController::depthCallback, this);

        // 订阅状态
        state_sub_ = nh_.subscribe("/uav1/mavros/state", 10, &RollingController::stateCallback, this);

        // 订阅位置
        pose_sub_ = nh_.subscribe("/uav1/mavros/local_position/pose", 10, &RollingController::poseCallback, this);

        // 发布高度 setpoint（仅 Z 轴）
        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
        
        // 发布彩色深度图像
        depth_color_pub_ = nh_.advertise<sensor_msgs::Image>("/depth_color", 10);

        // 参数
        nh_.param("rolling_height", rolling_height_, 0.1);   // 贴地高度 (米)
        nh_.param("avoid_height", avoid_height_, 0.8);       // 避障高度 (米)
        nh_.param("rc_channel_switch", rc_channel_switch_, 7); // 控制开关的通道（默认 CH7）
        nh_.param("obstacle_threshold", obstacle_threshold_, 1000); // 小于1000mm的点视为障碍物
        nh_.param("ratio_threshold", ratio_threshold_, 0.3);   // 30%的点视为有障碍物
        nh_.param("depth_image_topic", depth_image_topic_, std::string("/camera/depth/image_rect_raw")); // 深度图像话题
        nh_.param("auto_height_control", auto_height_control_, true); // 是否自动控高

        current_state_ = DISABLE;
        obstacle_ahead_ = false;
        depth_image_received_ = false;
        last_print_time_ = ros::Time::now();
    }

    void run() {
        ros::Rate rate(20); // 20 Hz

        while (ros::ok()) {
            executeControl();
            printStatus();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
        if (msg->channels.size() > rc_channel_switch_) {
            int ch_val = msg->channels[rc_channel_switch_];
            // 三段开关：低 = DISABLE, 高 = ROLLING_AUTO
            if (ch_val > 1750) {
                current_state_ = ROLLING_AUTO;
            } else {
                current_state_ = DISABLE;
            }
        }
    }

    // 状态回调
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        armed_ = msg->armed;
        mode_ = msg->mode;
    }

    // 位置回调
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_z_ = msg->pose.position.z;
    }

    // 深度图障碍检测
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_mat = cv_ptr->image;

            int rows = depth_mat.rows; // 图像高度
            int cols = depth_mat.cols; // 图像宽度

            // 将图像分成三个区域：左、中、右
            int region_width = cols / 3;    // 每个区域的宽度
            int check_height = rows / 5;    // 检查区域的高度（图像中间部分）
            int start_row = 2 * rows / 5;   // 开始检查的行（避开天空和地面）

            // 定义三个检查区域
            cv::Rect left_roi(0, start_row, region_width, check_height);                 // 左侧区域
            cv::Rect center_roi(region_width, start_row, region_width, check_height);    // 中间区域
            cv::Rect right_roi(2 * region_width, start_row, region_width, check_height); // 右侧区域

            // 计算每个区域的平均深度
            cv::Mat left_region = depth_mat(left_roi);
            cv::Mat center_region = depth_mat(center_roi);
            cv::Mat right_region = depth_mat(right_roi);

            // 把区域内的0值（无效值）排除在外
            left_region.setTo(0, left_region <= 300);
            center_region.setTo(0, center_region <= 300);
            right_region.setTo(0, right_region <= 300);

            // 计算每个区域内小于阈值的点的比例
            double left_obstacle_ratio = (double)cv::countNonZero(left_region < obstacle_threshold_) / (region_width * check_height);
            double center_obstacle_ratio = (double)cv::countNonZero(center_region < obstacle_threshold_) / (region_width * check_height);
            double right_obstacle_ratio = (double)cv::countNonZero(right_region < obstacle_threshold_) / (region_width * check_height);

            // 判断是否有障碍物
            obstacle_ahead_ = (center_obstacle_ratio > ratio_threshold_);

            // 将原本的深度图像转换成彩色，标记三个区域，并把三个区域内的障碍物比例显示出来，同时显示该区域是否有障碍物，把转换后的图像发布成新话题
            cv::Mat depth_color;
            double min_val, max_val;
            cv::minMaxLoc(depth_mat, &min_val, &max_val);
            depth_mat.convertTo(depth_color, CV_8U, 255.0 / max_val);
            cv::applyColorMap(depth_color, depth_color, cv::COLORMAP_JET);
            // 标记区域
            cv::rectangle(depth_color, left_roi, cv::Scalar(255, 255, 255), 1);
            cv::rectangle(depth_color, center_roi, cv::Scalar(255, 255, 255), 1);
            cv::rectangle(depth_color, right_roi, cv::Scalar(255, 255, 255), 1);
            // 在三个标记区域内选取三个点,标记出来，并打印其数值
            cv::Point left_point(region_width / 2, start_row + check_height / 2);
            cv::Point center_point(region_width + region_width / 2, start_row + check_height / 2);
            cv::Point right_point(2 * region_width + region_width / 2, start_row + check_height / 2);
            uint16_t left_depth = depth_mat.at<uint16_t>(left_point);
            uint16_t center_depth = depth_mat.at<uint16_t>(center_point);
            uint16_t right_depth = depth_mat.at<uint16_t>(right_point);
            cv::circle(depth_color, left_point, 5, cv::Scalar(0, 0, 0), -1);
            cv::circle(depth_color, center_point, 5, cv::Scalar(0, 0, 0), -1);
            cv::circle(depth_color, right_point, 5, cv::Scalar(0, 0, 0), -1);
            cv::putText(depth_color, cv::format("%d mm", left_depth), left_point, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
            cv::putText(depth_color, cv::format("%d mm", center_depth), center_point, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
            cv::putText(depth_color, cv::format("%d mm", right_depth), right_point, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
            // 显示障碍物比例和状态
            std::string left_text = cv::format("Left: %.1f%%", left_obstacle_ratio * 100);
            std::string center_text = cv::format("Center: %.1f%%", center_obstacle_ratio * 100);
            std::string right_text = cv::format("Right: %.1f%%", right_obstacle_ratio * 100);
            cv::putText(depth_color, left_text, cv::Point(10, start_row - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
            cv::putText(depth_color, center_text, cv::Point(region_width + 10, start_row - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
            cv::putText(depth_color, right_text, cv::Point(2 * region_width + 10, start_row - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
            // 显示障碍物状态
            std::string center_status = obstacle_ahead_ ? "Blocked" : "Clear";
            cv::putText(depth_color, center_status, cv::Point(region_width + 10, start_row + check_height + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, obstacle_ahead_ ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 1);
            // 发布彩色深度图像
            sensor_msgs::ImagePtr depth_color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_color).toImageMsg();
            depth_color_pub_.publish(depth_color_msg);

            depth_image_received_ = true;
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("深度图像转换错误: %s", e.what());
        }
    }

    void executeControl() {
        if (current_state_ == DISABLE) {
            // 不发布任何 setpoint，完全交由遥控器控制
            return;
        }

        if (!armed_) {
            // 未解锁，不发布任何 setpoint
            // 获取当前高度
            ground_height_ = current_z_;
            return;
        }
        
        // ROLLING_AUTO: 自动控高
        if (auto_height_control_) {
            double target_z_ = obstacle_ahead_ ? avoid_height_ : rolling_height_;
        } else {
            // 手动控高
            double target_z_ = ground_height_ + 0.1;
        }

        // 发布仅包含 Z 轴的位置 setpoint
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        // 注意：PZ 未被忽略 → 飞控将跟踪此高度

        target.position.z = target_z_;
        setpoint_pub_.publish(target);
    }

    void printStatus() {
        // 获取当前时间
        ros::Time current_time = ros::Time::now();
        
        // 检查是否已经过了1秒
        if (current_time - last_print_time_ < ros::Duration(1.0)) {
            return;
        }
        
        // 更新上次打印时间
        last_print_time_ = current_time;
        
        // 打印状态信息
        ROS_INFO("==================== Rolling Controller Status ====================");
        
        // 打印当前状态
        std::string state_str = (current_state_ == DISABLE) ? "DISABLE" : "ROLLING_AUTO";
        ROS_INFO("Current State: [%s]", state_str.c_str());
        
        // 打印深度图像接收状态
        ROS_INFO("Depth Image Received: [%s]", depth_image_received_ ? "Yes" : "No");
        
        // 打印障碍物检测状态
        ROS_INFO("Obstacle Ahead: [%s]", obstacle_ahead_ ? "Yes" : "No");
        
        // 打印高度信息
        ROS_INFO("Target Height: [%.3f] [m]", target_z_);
        ROS_INFO("Rolling Height: [%.3f] [m]", rolling_height_);
        ROS_INFO("Avoid Height: [%.3f] [m]", avoid_height_);
        
        // 打印自动控高状态
        ROS_INFO("Auto Height Control: [%s]", auto_height_control_ ? "Enabled" : "Disabled");
        
        // 打印障碍物检测参数
        ROS_INFO("Obstacle Threshold: [%d] [mm]", obstacle_threshold_);
        ROS_INFO("Ratio Threshold: [%.2f] [%%]", ratio_threshold_ * 100);
        
        // 打印RC通道信息
        ROS_INFO("RC Channel Switch: [%d]", rc_channel_switch_);
        
        // 打印深度图像话题
        ROS_INFO("Depth Image Topic: [%s]", depth_image_topic_.c_str());
        
        ROS_INFO("==============================================================");
    }

    // ========== 成员变量 ==========
    ros::NodeHandle nh_;
    ros::Subscriber rc_sub_, depth_sub_, state_sub_, pose_sub_;
    ros::Publisher setpoint_pub_, depth_color_pub_;

    State current_state_;

    // 无人机状态
    bool armed_;
    std::string mode_;

    // 当前位置
    geometry_msgs::PoseStamped current_pose_;

    bool obstacle_ahead_;
    bool depth_image_received_;
    double current_z_; // 当前高度
    double ground_height_; // 地面高度
    double target_z_; // 目标高度
    ros::Time last_print_time_; // 上次打印时间

    // 参数
    double rolling_height_;
    double avoid_height_;
    int rc_channel_switch_;
    int obstacle_threshold_;
    double ratio_threshold_;
    std::string depth_image_topic_;
    bool auto_height_control_; // 是否自动控高
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rolling_controller");
    RollingController controller;
    controller.run();
    return 0;
}
