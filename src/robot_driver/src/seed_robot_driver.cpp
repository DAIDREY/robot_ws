// src/seed_robot_driver.cpp
#include "seed_robot_driver.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_driver
{

// 静态常量定义
const std::vector<std::string> SeedRobotDriver::JOINT_NAMES = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
};

const double SeedRobotDriver::DEFAULT_JOINT_TOLERANCE = 0.01;  // 弧度
const double SeedRobotDriver::DEFAULT_POSITION_TOLERANCE = 1.0; // 毫米

SeedRobotDriver::SeedRobotDriver(const rclcpp::NodeOptions& options)
    : Node("seed_robot_driver", options), robot_state_(RobotState::DISCONNECTED)
{
    // 初始化参数
    initializeParameters();
    
    // 创建组件
    serial_comm_ = std::make_unique<SerialCommunication>(this->get_logger());
    command_builder_ = std::make_unique<CommandBuilder>();
    
    // 设置状态回调
    serial_comm_->setStatusCallback(
        std::bind(&SeedRobotDriver::statusCallback, this, std::placeholders::_1));
    
    // 初始化ROS接口
    initializePublishers();
    initializeSubscribers();
    initializeServices();
    initializeTimers();
    
    // 初始化状态
    memset(&current_status_, 0, sizeof(current_status_));
    
    RCLCPP_INFO(this->get_logger(), "SEED机器人驱动器已初始化");
}

SeedRobotDriver::~SeedRobotDriver()
{
    disconnect();
}

void SeedRobotDriver::initializeParameters()
{
    // 声明参数
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("max_joint_velocity", 30.0);  // 度/秒
    this->declare_parameter("max_cartesian_velocity", 1000.0);  // mm/分钟
    this->declare_parameter("auto_connect", true);
    this->declare_parameter("status_publish_rate", 50.0);  // Hz
    
    // 工作空间限制参数
    this->declare_parameter("workspace_min_x", -500.0);
    this->declare_parameter("workspace_min_y", -500.0);
    this->declare_parameter("workspace_min_z", 0.0);
    this->declare_parameter("workspace_max_x", 500.0);
    this->declare_parameter("workspace_max_y", 500.0);
    this->declare_parameter("workspace_max_z", 800.0);
    
    // 关节限制参数
    this->declare_parameter("joint_min_angles", std::vector<double>{-150.0, -65.0, -188.0, -167.0, -34.0, -270.0});
    this->declare_parameter("joint_max_angles", std::vector<double>{150.0, 110.0, 27.0, 167.0, 208.0, 270.0});
    
    // 获取参数值
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    max_joint_velocity_ = this->get_parameter("max_joint_velocity").as_double();
    max_cartesian_velocity_ = this->get_parameter("max_cartesian_velocity").as_double();
    
    // 设置工作空间限制
    workspace_min_.x = this->get_parameter("workspace_min_x").as_double();
    workspace_min_.y = this->get_parameter("workspace_min_y").as_double();
    workspace_min_.z = this->get_parameter("workspace_min_z").as_double();
    workspace_max_.x = this->get_parameter("workspace_max_x").as_double();
    workspace_max_.y = this->get_parameter("workspace_max_y").as_double();
    workspace_max_.z = this->get_parameter("workspace_max_z").as_double();
    
    // 设置关节限制
    auto joint_min_vec = this->get_parameter("joint_min_angles").as_double_array();
    auto joint_max_vec = this->get_parameter("joint_max_angles").as_double_array();
    
    for (size_t i = 0; i < 6 && i < joint_min_vec.size(); ++i) {
        joint_min_.angles[i] = joint_min_vec[i];
    }
    for (size_t i = 0; i < 6 && i < joint_max_vec.size(); ++i) {
        joint_max_.angles[i] = joint_max_vec[i];
    }
    
    // 自动连接
    if (this->get_parameter("auto_connect").as_bool()) {
        // 延迟连接，确保节点完全初始化
        auto timer = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (connect()) {
                    RCLCPP_INFO(this->get_logger(), "自动连接成功");
                } else {
                    RCLCPP_WARN(this->get_logger(), "自动连接失败");
                }
            }
        );
        // 单次定时器
        timer->cancel();
    }
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
        std::bind(&SeedRobotDriver::jointTrajectoryCallback, this, std::placeholders::_1));
    
    pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose_target", 10,
        std::bind(&SeedRobotDriver::poseTargetCallback, this, std::placeholders::_1));
}

void SeedRobotDriver::initializeServices()
{
    connect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "connect",
        std::bind(&SeedRobotDriver::handleConnectService, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "disconnect",
        std::bind(&SeedRobotDriver::handleDisconnectService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "emergency_stop",
        std::bind(&SeedRobotDriver::handleEmergencyStopService, this,
                  std::placeholders::_1, std::placeholders::_2));
}

void SeedRobotDriver::initializeTimers()
{
    double publish_rate = this->get_parameter("status_publish_rate").as_double();
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    
    status_timer_ = this->create_wall_timer(
        period, std::bind(&SeedRobotDriver::statusPublishTimer, this));
}

bool SeedRobotDriver::connect()
{
    if (isConnected()) {
        RCLCPP_WARN(this->get_logger(), "机器人已经连接");
        return true;
    }
    
    if (serial_comm_->connect(serial_port_, baudrate_)) {
        // 启动状态接收
        serial_comm_->startStatusReceiving();
        
        // 设置状态信息模式为全部信息
        serial_comm_->sendSetStatusMode(2);
        
        robot_state_ = RobotState::CONNECTED;
        RCLCPP_INFO(this->get_logger(), "机器人连接成功: %s", serial_port_.c_str());
        
        // 延迟一段时间后设置为READY状态
        auto timer = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                if (robot_state_ == RobotState::CONNECTED) {
                    robot_state_ = RobotState::READY;
                    RCLCPP_INFO(this->get_logger(), "机器人就绪");
                }
            }
        );
        
        return true;
    } else {
        robot_state_ = RobotState::DISCONNECTED;
        RCLCPP_ERROR(this->get_logger(), "机器人连接失败");
        return false;
    }
}

void SeedRobotDriver::disconnect()
{
    if (serial_comm_) {
        serial_comm_->stopStatusReceiving();
        serial_comm_->disconnect();
    }
    
    robot_state_ = RobotState::DISCONNECTED;
    RCLCPP_INFO(this->get_logger(), "机器人已断开连接");
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
    if (robot_state_ != RobotState::READY) {
        RCLCPP_ERROR(this->get_logger(), "机器人未就绪，无法执行运动");
        return false;
    }
    
    if (!isInWorkspace(position)) {
        RCLCPP_ERROR(this->get_logger(), "目标位置超出工作空间限制");
        return false;
    }
    
    if (speed > max_cartesian_velocity_) {
        RCLCPP_WARN(this->get_logger(), "速度超出限制，使用最大速度: %.1f", max_cartesian_velocity_);
        speed = max_cartesian_velocity_;
    }
    
    auto command = command_builder_->buildLinearMove(position, orientation, 1500, speed);
    
    if (serial_comm_->sendCommand(command)) {
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
    if (robot_state_ != RobotState::READY) {
        RCLCPP_ERROR(this->get_logger(), "机器人未就绪，无法执行运动");
        return false;
    }
    
    if (!isValidJointAngles(joint_angles)) {
        RCLCPP_ERROR(this->get_logger(), "关节角度超出限制");
        return false;
    }
    
    if (speed > max_joint_velocity_) {
        RCLCPP_WARN(this->get_logger(), "关节速度超出限制，使用最大速度: %.1f", max_joint_velocity_);
        speed = max_joint_velocity_;
    }
    
    auto command = command_builder_->buildJointMove(joint_angles, 0, speed, 1500);
    
    if (serial_comm_->sendCommand(command)) {
        robot_state_ = RobotState::MOVING;
        RCLCPP_INFO(this->get_logger(), "发送关节运动指令");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "发送关节运动指令失败");
        return false;
    }
}

bool SeedRobotDriver::moveSingleAxis(uint8_t axis, double angle_increment, double speed)
{
    if (robot_state_ != RobotState::READY) {
        RCLCPP_ERROR(this->get_logger(), "机器人未就绪，无法执行运动");
        return false;
    }
    
    if (axis >= 6) {
        RCLCPP_ERROR(this->get_logger(), "无效的轴号: %d", axis);
        return false;
    }
    
    if (speed > max_joint_velocity_) {
        speed = max_joint_velocity_;
    }
    
    auto command = command_builder_->buildSingleAxisMove(axis, angle_increment, 0, speed, 1500);
    
    if (serial_comm_->sendCommand(command)) {
        robot_state_ = RobotState::MOVING;
        RCLCPP_INFO(this->get_logger(), "发送单轴运动指令: 轴%d, 角度%.2f°", axis, angle_increment);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "发送单轴运动指令失败");
        return false;
    }
}

bool SeedRobotDriver::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (robot_state_ != RobotState::READY) {
        RCLCPP_ERROR(this->get_logger(), "机器人未就绪，无法执行轨迹");
        return false;
    }
    
    if (!validateTrajectory(trajectory)) {
        RCLCPP_ERROR(this->get_logger(), "轨迹验证失败");
        return false;
    }
    
    robot_state_ = RobotState::MOVING;
    
    // 执行轨迹中的每个点
    for (const auto& point : trajectory.points) {
        if (point.positions.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "轨迹点关节数量错误");
            robot_state_ = RobotState::ERROR;
            return false;
        }
        
        JointAngles joint_angles;
        for (size_t i = 0; i < 6; ++i) {
            joint_angles.angles[i] = point.positions[i] * 180.0 / M_PI;  // 转换为度
        }
        
        // 计算速度
        double speed = max_joint_velocity_;
        if (!point.velocities.empty()) {
            double max_vel = 0.0;
            for (double vel : point.velocities) {
                max_vel = std::max(max_vel, std::abs(vel * 180.0 / M_PI));
            }
            speed = std::min(max_vel, max_joint_velocity_);
        }
        
        auto command = command_builder_->buildJointMove(joint_angles, 1, speed, 1500);
        
        if (!serial_comm_->sendCommand(command)) {
            RCLCPP_ERROR(this->get_logger(), "发送轨迹点失败");
            robot_state_ = RobotState::ERROR;
            return false;
        }
        
        // 简单的时间延迟（实际应用中应该使用更精确的时序控制）
        if (!point.time_from_start.nanosec == 0 && point.time_from_start.sec == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            auto duration = rclcpp::Duration(point.time_from_start);
            std::this_thread::sleep_for(std::chrono::milliseconds(duration.nanoseconds() / 1000000));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
    return true;
}

bool SeedRobotDriver::stopMotion()
{
    if (!isConnected()) {
        return false;
    }
    
    // 这里可以发送停止指令，或者简单地更新状态
    robot_state_ = RobotState::READY;
    RCLCPP_INFO(this->get_logger(), "运动已停止");
    return true;
}

bool SeedRobotDriver::emergencyStop()
{
    if (!isConnected()) {
        return false;
    }
    
    if (serial_comm_->sendEmergencyStop()) {
        robot_state_ = RobotState::EMERGENCY_STOP;
        RCLCPP_WARN(this->get_logger(), "急停已激活");
        return true;
    }
    return false;
}

bool SeedRobotDriver::emergencyRelease()
{
    if (!isConnected()) {
        return false;
    }
    
    if (serial_comm_->sendEmergencyRelease()) {
        robot_state_ = RobotState::READY;
        RCLCPP_INFO(this->get_logger(), "急停已解除");
        return true;
    }
    return false;
}

bool SeedRobotDriver::resetRobot()
{
    if (!isConnected()) {
        return false;
    }
    
    if (serial_comm_->sendRobotReset()) {
        robot_state_ = RobotState::CONNECTED;
        RCLCPP_INFO(this->get_logger(), "机器人已复位");
        
        // 延迟后设置为READY
        auto timer = this->create_wall_timer(
            std::chrono::seconds(3),
            [this]() {
                robot_state_ = RobotState::READY;
                RCLCPP_INFO(this->get_logger(), "机器人复位完成，已就绪");
            }
        );
        
        return true;
    }
    return false;
}

bool SeedRobotDriver::calibrateRobot()
{
    if (!isConnected()) {
        return false;
    }
    
    auto command = command_builder_->buildAutoCalibration();
    if (serial_comm_->sendCommand(command)) {
        RCLCPP_INFO(this->get_logger(), "机器人自动校准已启动");
        return true;
    }
    return false;
}

bool SeedRobotDriver::setWorkspaceOrigin(const Position3D& origin)
{
    if (!isConnected()) {
        return false;
    }
    
    auto command = command_builder_->buildSetWorkspaceOrigin(origin);
    if (serial_comm_->sendCommand(command)) {
        RCLCPP_INFO(this->get_logger(), "工作台原点已设置: (%.1f, %.1f, %.1f)", 
                   origin.x, origin.y, origin.z);
        return true;
    }
    return false;
}

// 状态查询方法
RobotStatus SeedRobotDriver::getCurrentStatus() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

Position3D SeedRobotDriver::getCurrentPosition() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return Position3D(current_status_.x, current_status_.y, current_status_.z);
}

JointAngles SeedRobotDriver::getCurrentJointAngles() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    JointAngles angles;
    for (size_t i = 0; i < 6; ++i) {
        angles.angles[i] = current_status_.joint_angles[i];
    }
    return angles;
}

// 安全检查方法
bool SeedRobotDriver::isInWorkspace(const Position3D& position) const
{
    return position.x >= workspace_min_.x && position.x <= workspace_max_.x &&
           position.y >= workspace_min_.y && position.y <= workspace_max_.y &&
           position.z >= workspace_min_.z && position.z <= workspace_max_.z;
}

bool SeedRobotDriver::isValidJointAngles(const JointAngles& angles) const
{
    for (size_t i = 0; i < 6; ++i) {
        if (angles.angles[i] < joint_min_.angles[i] || angles.angles[i] > joint_max_.angles[i]) {
            RCLCPP_WARN(this->get_logger(), "关节%zu角度超限: %.2f° (限制: %.2f° ~ %.2f°)", 
                       i, angles.angles[i], joint_min_.angles[i], joint_max_.angles[i]);
            return false;
        }
    }
    return true;
}

void SeedRobotDriver::setWorkspaceLimits(const Position3D& min_pos, const Position3D& max_pos)
{
    workspace_min_ = min_pos;
    workspace_max_ = max_pos;
    RCLCPP_INFO(this->get_logger(), "工作空间限制已更新");
}

void SeedRobotDriver::setJointLimits(const JointAngles& min_angles, const JointAngles& max_angles)
{
    joint_min_ = min_angles;
    joint_max_ = max_angles;
    RCLCPP_INFO(this->get_logger(), "关节限制已更新");
}

// 回调函数实现
void SeedRobotDriver::statusCallback(const RobotStatus& status)
{
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = status;
    }
    
    // 更新机器人状态
    updateRobotState();
}

void SeedRobotDriver::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "收到关节轨迹，包含 %zu 个点", msg->points.size());
    
    if (!executeTrajectory(*msg)) {
        RCLCPP_ERROR(this->get_logger(), "执行关节轨迹失败");
    }
}

void SeedRobotDriver::poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Position3D target_pos(
        msg->pose.position.x * 1000.0,  // 转换为毫米
        msg->pose.position.y * 1000.0,
        msg->pose.position.z * 1000.0
    );
    
    // 简化的姿态处理（实际应用中需要转换四元数）
    Orientation target_ori(0.0, 0.0, 0.0);
    
    RCLCPP_INFO(this->get_logger(), "收到位置目标: (%.1f, %.1f, %.1f)", 
               target_pos.x, target_pos.y, target_pos.z);
    
    if (!moveToPosition(target_pos, target_ori)) {
        RCLCPP_ERROR(this->get_logger(), "移动到目标位置失败");
    }
}

void SeedRobotDriver::statusPublishTimer()
{
    publishJointState();
    publishCurrentPose();
    publishRobotStatus();
}

// 发布方法
void SeedRobotDriver::publishJointState()
{
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.name = JOINT_NAMES;
    
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        joint_state_msg.position.resize(6);
        joint_state_msg.velocity.resize(6);
        
        for (size_t i = 0; i < 6; ++i) {
            joint_state_msg.position[i] = current_status_.joint_angles[i] * M_PI / 180.0;  // 转换为弧度
            joint_state_msg.velocity[i] = 0.0;  // TODO: 实现速度计算
        }
    }
    
    joint_state_pub_->publish(joint_state_msg);
}

void SeedRobotDriver::publishCurrentPose()
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        pose_msg.pose.position.x = current_status_.x / 1000.0;  // 转换为米
        pose_msg.pose.position.y = current_status_.y / 1000.0;
        pose_msg.pose.position.z = current_status_.z / 1000.0;
        
        // 简化的姿态设置（实际应用中需要从B0, B1, W计算四元数）
        pose_msg.pose.orientation.w = 1.0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
    }
    
    pose_pub_->publish(pose_msg);
}

void SeedRobotDriver::publishRobotStatus()
{
    std_msgs::msg::String status_msg;
    
    switch (robot_state_) {
        case RobotState::DISCONNECTED:
            status_msg.data = "DISCONNECTED";
            break;
        case RobotState::CONNECTED:
            status_msg.data = "CONNECTED";
            break;
        case RobotState::READY:
            status_msg.data = "READY";
            break;
        case RobotState::MOVING:
            status_msg.data = "MOVING";
            break;
        case RobotState::ERROR:
            status_msg.data = "ERROR";
            break;
        case RobotState::EMERGENCY_STOP:
            status_msg.data = "EMERGENCY_STOP";
            break;
        default:
            status_msg.data = "UNKNOWN";
            break;
    }
    
    status_pub_->publish(status_msg);
}

void SeedRobotDriver::updateRobotState()
{
    // 根据当前状态和错误代码更新机器人状态
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    if (current_status_.error_code != 0) {
        robot_state_ = RobotState::ERROR;
    } else if (robot_state_ == RobotState::MOVING) {
        // 检查是否运动完成（简化检查）
        // 实际应用中需要更复杂的运动状态检测
        robot_state_ = RobotState::READY;
    }
}

bool SeedRobotDriver::validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.joint_names.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "轨迹关节数量错误: %zu", trajectory.joint_names.size());
        return false;
    }
    
    for (const auto& point : trajectory.points) {
        if (point.positions.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "轨迹点位置数量错误");
            return false;
        }
        
        JointAngles angles;
        for (size_t i = 0; i < 6; ++i) {
            angles.angles[i] = point.positions[i] * 180.0 / M_PI;  // 转换为度
        }
        
        if (!isValidJointAngles(angles)) {
            return false;
        }
    }
    
    return true;
}

// 服务回调实现
void SeedRobotDriver::handleConnectService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    response->success = connect();
    if (response->success) {
        response->message = "机器人连接成功";
    } else {
        response->message = "机器人连接失败";
    }
}

void SeedRobotDriver::handleDisconnectService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    disconnect();
    response->success = true;
    response->message = "机器人已断开连接";
}

void SeedRobotDriver::handleEmergencyStopService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    response->success = emergencyStop();
    if (response->success) {
        response->message = "急停已激活";
    } else {
        response->message = "急停激活失败";
    }
}

} // namespace robot_driver