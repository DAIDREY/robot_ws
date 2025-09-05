// include/robot_driver/seed_robot_driver.hpp
#ifndef ROBOT_DRIVER_SEED_ROBOT_DRIVER_HPP
#define ROBOT_DRIVER_SEED_ROBOT_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <mutex>
#include <atomic>

#include "serial_communication.hpp"
#include "command_builder.hpp"

namespace robot_driver
{

enum class RobotState
{
    DISCONNECTED,
    CONNECTED,
    READY,
    MOVING,
    ERROR,
    EMERGENCY_STOP
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
    bool setWorkspaceOrigin(const Position3D& origin);
    
    // 状态查询
    RobotStatus getCurrentStatus() const;
    Position3D getCurrentPosition() const;
    JointAngles getCurrentJointAngles() const;
    
    // 安全功能
    bool isInWorkspace(const Position3D& position) const;
    bool isValidJointAngles(const JointAngles& angles) const;
    void setWorkspaceLimits(const Position3D& min_pos, const Position3D& max_pos);
    void setJointLimits(const JointAngles& min_angles, const JointAngles& max_angles);

private:
    // 初始化
    void initializeParameters();
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();
    void initializeTimers();
    
    // 回调函数
    void statusCallback(const RobotStatus& status);
    void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void statusPublishTimer();
    
    // 服务回调
    void handleConnectService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleDisconnectService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleEmergencyStopService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // 内部方法
    void publishJointState();
    void publishCurrentPose();
    void publishRobotStatus();
    void updateRobotState();
    bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    
    // 参数
    std::string serial_port_;
    int baudrate_;
    double max_joint_velocity_;
    double max_cartesian_velocity_;
    Position3D workspace_min_, workspace_max_;
    JointAngles joint_min_, joint_max_;
    
    // 组件
    std::unique_ptr<SerialCommunication> serial_comm_;
    std::unique_ptr<CommandBuilder> command_builder_;
    
    // 状态
    std::atomic<RobotState> robot_state_;
    RobotStatus current_status_;
    mutable std::mutex status_mutex_;
    
    // ROS接口
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
    
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 常量
    static const std::vector<std::string> JOINT_NAMES;
    static const double DEFAULT_JOINT_TOLERANCE;
    static const double DEFAULT_POSITION_TOLERANCE;
};

} // namespace robot_driver

#endif // ROBOT_DRIVER_SEED_ROBOT_DRIVER_HPP