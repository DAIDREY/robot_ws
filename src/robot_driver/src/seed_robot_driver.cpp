// src/robot_driver/src/seed_robot_driver.cpp
#include "seed_robot_driver.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace robot_driver
{

const std::vector<std::string> SeedRobotDriver::JOINT_NAMES = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
};

SeedRobotDriver::SeedRobotDriver(const rclcpp::NodeOptions& options)
    : Node("seed_robot_driver", options), robot_state_(RobotState::DISCONNECTED)
{
    initializeParameters();
    
    // 创建组件
    serial_comm_ = std::make_unique<SerialCommunication>(this->get_logger());
    
    // 设置状态回调
    serial_comm_->setStatusCallback(
        std::bind(&SeedRobotDriver::statusCallback, this, std::placeholders::_1));
    
    // 初始化运动检测变量
    last_stable_time_ = std::chrono::steady_clock::now();
    
    // 初始化ROS接口
    initializePublishers();
    initializeSubscribers();
    initializeServices();
    initializeTimers();
    
    // 启动后直接连接
    if (connect()) {
        RCLCPP_INFO(this->get_logger(), "SEED机器人驱动器已初始化并连接成功");
    } else {
        RCLCPP_ERROR(this->get_logger(), "SEED机器人驱动器初始化失败，连接串口失败");
    }
}

SeedRobotDriver::~SeedRobotDriver()
{
    disconnect();
}

void SeedRobotDriver::initializeParameters()
{
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("max_joint_velocity", 30.0);
    this->declare_parameter("max_cartesian_velocity", 100.0);
    this->declare_parameter("status_publish_rate", 10.0);
    
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    max_joint_velocity_ = this->get_parameter("max_joint_velocity").as_double();
    max_cartesian_velocity_ = this->get_parameter("max_cartesian_velocity").as_double();
}

void SeedRobotDriver::initializePublishers()
{
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);
}

void SeedRobotDriver::initializeSubscribers()
{
    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory", 10,
        std::bind(&SeedRobotDriver::trajectoryCallback, this, std::placeholders::_1));
    
    pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose_target", 10,
        std::bind(&SeedRobotDriver::poseTargetCallback, this, std::placeholders::_1));
}

void SeedRobotDriver::initializeServices()
{
    connect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "connect", std::bind(&SeedRobotDriver::connectServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));
    
    disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "disconnect", std::bind(&SeedRobotDriver::disconnectServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));
}

void SeedRobotDriver::initializeTimers()
{
    // 现有的状态发布定时器
    double rate = this->get_parameter("status_publish_rate").as_double();
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate));
    
    status_timer_ = this->create_wall_timer(
        period, std::bind(&SeedRobotDriver::publishStatus, this));

    auto motion_check_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0));  // 1Hz检测
    
    motion_check_timer_ = this->create_wall_timer(
        motion_check_period, std::bind(&SeedRobotDriver::checkMotionComplete, this));
}

void SeedRobotDriver::checkMotionComplete()
{
    if (robot_state_ != RobotState::MOVING) {
        return;  // 不在运动状态，无需检测
    }
    
    static auto last_check_time = std::chrono::steady_clock::now();
    static RobotStatus last_status;
    static bool first_check = true;
    static auto stable_start_time = std::chrono::steady_clock::now();
    
    auto current_time = std::chrono::steady_clock::now();
    
    // 获取当前状态
    RobotStatus current_status;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status = current_status_;
    }
    
    if (first_check) {
        last_status = current_status;
        last_check_time = current_time;
        stable_start_time = current_time;
        first_check = false;
        return;
    }
    
    // 检查位置和角度变化
    double position_threshold = 0.1;  // mm
    double angle_threshold = 0.01;    // 度
    
    bool xyz_stable = (std::abs(current_status.x - last_status.x) < position_threshold) &&
                     (std::abs(current_status.y - last_status.y) < position_threshold) &&
                     (std::abs(current_status.z - last_status.z) < position_threshold);
    
    bool joints_stable = true;
    for (int i = 0; i < 6; i++) {
        if (std::abs(current_status.joint_angles[i] - last_status.joint_angles[i]) > angle_threshold) {
            joints_stable = false;
            break;
        }
    }
    
    if (xyz_stable && joints_stable) {
        // 检查稳定时间是否达到2秒
        if (std::chrono::duration<double>(current_time - stable_start_time).count() >= 2.0) {
            robot_state_ = RobotState::READY;
            first_check = true;  // 重置检测状态
            RCLCPP_INFO(this->get_logger(), "✅ 运动完成检测：机械臂已稳定");
        }
    } else {
        // 重新开始稳定计时
        stable_start_time = current_time;
    }
    
    last_status = current_status;
    last_check_time = current_time;
}


bool SeedRobotDriver::connect()
{
    if (serial_comm_->connect(serial_port_, baudrate_)) {
        serial_comm_->startStatusReceiving();
        robot_state_ = RobotState::CONNECTED;
        RCLCPP_INFO(this->get_logger(), "机器人连接成功");
        return true;
    } else {
        robot_state_ = RobotState::DISCONNECTED;
        RCLCPP_ERROR(this->get_logger(), "机器人连接失败");
        return false;
    }
}

void SeedRobotDriver::disconnect()
{
    if (serial_comm_ && serial_comm_->isConnected()) {
        serial_comm_->stopStatusReceiving();
        serial_comm_->disconnect();
        robot_state_ = RobotState::DISCONNECTED;
        RCLCPP_INFO(this->get_logger(), "机器人已断开连接");
    }
}

bool SeedRobotDriver::isConnected() const
{
    return serial_comm_ && serial_comm_->isConnected();
}

RobotState SeedRobotDriver::getRobotState() const
{
    return robot_state_;
}

bool SeedRobotDriver::moveToPosition(const Position3D& position, const Orientation& orientation, double speed)
{
    if (robot_state_ != RobotState::READY && robot_state_ != RobotState::CONNECTED) {
        RCLCPP_ERROR(this->get_logger(), "机器人未就绪，无法执行运动");
        return false;
    }
    
    if (speed > max_cartesian_velocity_) {
        speed = max_cartesian_velocity_;
    }
    
    if (serial_comm_->sendLinearMove(position, orientation, speed, 1500)) {
        robot_state_ = RobotState::MOVING;
        RCLCPP_INFO(this->get_logger(), "发送直线运动指令: (%.1f, %.1f, %.1f)", 
                   position.x, position.y, position.z);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "发送运动指令失败");
        return false;
    }
}

bool SeedRobotDriver::moveJoints(const JointAngles& joint_angles, double speed)
{
    if (robot_state_ != RobotState::READY && robot_state_ != RobotState::CONNECTED) {
        RCLCPP_ERROR(this->get_logger(), "机器人未就绪，无法执行运动");
        return false;
    }
    
    if (speed > max_joint_velocity_) {
        speed = max_joint_velocity_;
    }
    
    if (serial_comm_->sendJointMove(joint_angles, speed, 0, 1500)) {
        robot_state_ = RobotState::MOVING;
        RCLCPP_INFO(this->get_logger(), "发送关节运动指令");
    } else {
        RCLCPP_ERROR(this->get_logger(), "发送关节运动指令失败");
        return false;
    }
}

bool SeedRobotDriver::waitForMotionComplete(double timeout_seconds, double position_threshold, double angle_threshold)
{
    RCLCPP_INFO(this->get_logger(), "开始运动完成检测，超时: %.1fs", timeout_seconds);
    
    auto start_time = std::chrono::steady_clock::now();
    auto last_check_time = start_time;
    
    // 获取初始状态
    RobotStatus initial_status;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        initial_status = current_status_;
    }
    
    RobotStatus last_status = initial_status;
    auto stable_start_time = start_time;
    
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < timeout_seconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 200ms检查一次
        
        // 每1秒检查一次变化
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_check_time).count() >= 1.0) {
            RobotStatus current_status;
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                current_status = current_status_;
            }
            
            // 检查XYZ变化
            bool xyz_stable = (std::abs(current_status.x - last_status.x) < position_threshold) &&
                             (std::abs(current_status.y - last_status.y) < position_threshold) &&
                             (std::abs(current_status.z - last_status.z) < position_threshold);
            
            // 检查关节角度变化
            bool joints_stable = true;
            for (int i = 0; i < 6; i++) {
                if (std::abs(current_status.joint_angles[i] - last_status.joint_angles[i]) > angle_threshold) {
                    joints_stable = false;
                    break;
                }
            }
            
            if (xyz_stable && joints_stable) {
                // 检查稳定时间是否达到1秒
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - stable_start_time).count() >= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "运动完成（位置稳定超过1秒）");
                    return true;
                }
            } else {
                // 重新开始稳定计时
                stable_start_time = std::chrono::steady_clock::now();
            }
            
            last_status = current_status;
            last_check_time = std::chrono::steady_clock::now();
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "运动完成检测超时（%.1fs）", timeout_seconds);
    return false;
}

void SeedRobotDriver::statusCallback(const RobotStatus& status)
{
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = status;
    }
    
    // 更新运动检测状态
    {
        std::lock_guard<std::mutex> lock(motion_check_mutex_);
        last_check_status_ = status;
        last_stable_time_ = std::chrono::steady_clock::now();
    }
    
    // 根据状态更新机器人状态
    if (robot_state_ == RobotState::CONNECTED) {
        robot_state_ = RobotState::READY;
    }
}

void SeedRobotDriver::publishStatus()
{
    if (robot_state_ == RobotState::DISCONNECTED) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 发布关节状态 - 运动时也发布
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.name = JOINT_NAMES;
    
    joint_state_msg.position.resize(6);
    for (int i = 0; i < 6; i++) {
        joint_state_msg.position[i] = current_status_.joint_angles[i] * M_PI / 180.0;
    }
    joint_state_pub_->publish(joint_state_msg);
    
    // 发布位置信息 - 运动时也发布
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.position.x = current_status_.x / 1000.0;
    pose_msg.pose.position.y = current_status_.y / 1000.0;
    pose_msg.pose.position.z = current_status_.z / 1000.0;
    
    // 添加姿态信息
    tf2::Quaternion q;
    q.setRPY(current_status_.orientation[0] * M_PI / 180.0,  // B0 → Roll
             current_status_.orientation[1] * M_PI / 180.0,  // B1 → Pitch  
             current_status_.orientation[2] * M_PI / 180.0); // W → Yaw
    pose_msg.pose.orientation = tf2::toMsg(q);
    
    pose_pub_->publish(pose_msg);
    
    // 发布机器人状态
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "State: " + stateToString(robot_state_) + 
                     ", PWM: " + std::to_string(current_status_.pwm_value) +
                     ", XYZ: (" + std::to_string(current_status_.x) + "," +
                                  std::to_string(current_status_.y) + "," + 
                                  std::to_string(current_status_.z) + ")";
    status_pub_->publish(status_msg);
}

void SeedRobotDriver::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty()) return;
    
    // 执行轨迹的第一个点
    if (msg->points[0].positions.size() == 6) {
        JointAngles joint_angles;
        for (size_t i = 0; i < 6; ++i) {
            joint_angles.angles[i] = msg->points[0].positions[i] * 180.0 / M_PI;
        }
        moveJoints(joint_angles, max_joint_velocity_);
    }
}

void SeedRobotDriver::poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Position3D position(
        msg->pose.position.x * 1000.0, // 转换为毫米
        msg->pose.position.y * 1000.0,
        msg->pose.position.z * 1000.0
    );
    
    // 从四元数中提取欧拉角
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // 转换为度
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
    
    // 创建姿态对象，映射到机器人的B0, B1, W轴
    Orientation orientation(
        roll,   // B0轴 (Roll)
        pitch,  // B1轴 (Pitch)  
        yaw     // W轴 (Yaw) - 这是第6关节!
    );
    
    RCLCPP_INFO(this->get_logger(), 
        "🎯 接收姿态指令: 位置(%.3f, %.3f, %.3f)mm, 姿态(%.1f°, %.1f°, %.1f°)",
        position.x, position.y, position.z, roll, pitch, yaw);
    
    moveToPosition(position, orientation, max_cartesian_velocity_);
}

void SeedRobotDriver::connectServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = connect();
    response->message = response->success ? "连接成功" : "连接失败";
}

void SeedRobotDriver::disconnectServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    disconnect();
    response->success = true;
    response->message = "已断开连接";
}

std::string SeedRobotDriver::stateToString(RobotState state)
{
    switch (state) {
        case RobotState::DISCONNECTED: return "DISCONNECTED";
        case RobotState::CONNECTED: return "CONNECTED";
        case RobotState::READY: return "READY";
        case RobotState::MOVING: return "MOVING";
        case RobotState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

} // namespace robot_driver