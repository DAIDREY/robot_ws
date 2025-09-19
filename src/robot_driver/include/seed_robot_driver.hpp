// include/robot_driver/seed_robot_driver.hpp
#ifndef ROBOT_DRIVER_SEED_ROBOT_DRIVER_HPP
#define ROBOT_DRIVER_SEED_ROBOT_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

#include "serial_communication.hpp"

namespace robot_driver
{

// 机器人状态枚举
enum class RobotState
{
    DISCONNECTED,
    CONNECTED,
    READY,
    MOVING,
    ERROR
}; 

class SeedRobotDriver : public rclcpp::Node
{
public:
    explicit SeedRobotDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SeedRobotDriver();
    
    // 连接管理
    bool connect();
    void disconnect();
    bool isConnected() const;
    RobotState getRobotState() const;
    
    // 运动控制
    bool moveToPosition(const Position3D& position, const Orientation& orientation, double speed = 1000.0);
    bool moveJoints(const JointAngles& joint_angles, double speed = 30.0);
    bool moveSingleAxis(uint8_t axis, double angle_increment, double speed = 30.0);
    bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    
    // 机器人控制
    bool stopMotion();
    bool emergencyStop();
    bool emergencyRelease();
    bool resetRobot();
    bool calibrateRobot();
    
    // 运动完成检测
    bool waitForMotionComplete(double timeout_seconds = 20.0, 
                              double position_threshold = 0.1, 
                              double angle_threshold = 0.01);

private:
    void initializeParameters();
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();
    void initializeTimers();
    
    // 回调函数
    void statusCallback(const RobotStatus& status);
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishStatus();
    
    // 服务回调
    void connectServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                               std_srvs::srv::Trigger::Response::SharedPtr response);
    void disconnectServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                  std_srvs::srv::Trigger::Response::SharedPtr response);
    void emergencyStopServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                     std_srvs::srv::Trigger::Response::SharedPtr response);
    
    // 工具函数
    sensor_msgs::msg::JointState createJointStateMessage(const RobotStatus& status);
    geometry_msgs::msg::PoseStamped createPoseMessage(const RobotStatus& status);
    std::string stateToString(RobotState state);
    
    // 参数变量
    std::string serial_port_;
    int baudrate_;
    double max_joint_velocity_;        // 度/秒
    double max_cartesian_velocity_;    // mm/分钟
    Position3D workspace_min_, workspace_max_;
    JointAngles joint_min_, joint_max_;
    
    // 组件
    std::unique_ptr<SerialCommunication> serial_comm_;
    
    // 状态变量
    std::atomic<RobotState> robot_state_;
    RobotStatus current_status_;
    mutable std::mutex status_mutex_;
    
    // 运动检测变量
    RobotStatus last_check_status_;
    std::chrono::steady_clock::time_point last_stable_time_;
    mutable std::mutex motion_check_mutex_;
    
    // ROS2 发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // ROS2 订阅者
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
    
    // ROS2 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    
    // ROS2 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 静态常量
    static const std::vector<std::string> JOINT_NAMES;
    static const double DEFAULT_JOINT_TOLERANCE;
    static const double DEFAULT_POSITION_TOLERANCE;
};

} // namespace robot_driver

#endif // ROBOT_DRIVER_SEED_ROBOT_DRIVER_HPP