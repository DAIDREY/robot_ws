#ifndef TRAJECTORY_OPTIMIZER_HPP
#define TRAJECTORY_OPTIMIZER_HPP

#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace motion_planner
{

class TrajectoryOptimizer
{
public:
    explicit TrajectoryOptimizer(rclcpp::Logger logger);
    ~TrajectoryOptimizer() = default;

    // 核心优化功能
    bool optimizeTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory, 
                           int max_points);
    
    bool smoothTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                         double smoothing_factor = 0.1);
    
    bool resampleTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                           int target_points);
    
    // 轨迹验证
    bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    
    bool checkVelocityLimits(const trajectory_msgs::msg::JointTrajectory& trajectory,
                            const std::vector<double>& max_velocities);
    
    bool checkAccelerationLimits(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                 const std::vector<double>& max_accelerations);
    
    // 轨迹分析
    double calculateTrajectoryLength(const trajectory_msgs::msg::JointTrajectory& trajectory);
    double calculateExecutionTime(const trajectory_msgs::msg::JointTrajectory& trajectory);
    std::vector<double> calculateJointDisplacements(const trajectory_msgs::msg::JointTrajectory& trajectory);
    
    // 时间参数化
    bool reparameterizeTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                                 double velocity_scaling_factor,
                                 double acceleration_scaling_factor);

private:
    rclcpp::Logger logger_;
    
    // 默认限制
    static constexpr double DEFAULT_MAX_VELOCITY = 1.0;        // rad/s
    static constexpr double DEFAULT_MAX_ACCELERATION = 2.0;    // rad/s^2
    static constexpr double MIN_TIME_STEP = 0.01;              // s
    
    // 内部辅助函数
    std::vector<size_t> selectOptimalPoints(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                           int target_points);
    
    double calculatePointDistance(const trajectory_msgs::msg::JointTrajectoryPoint& p1,
                                 const trajectory_msgs::msg::JointTrajectoryPoint& p2);
    
    void computeVelocities(trajectory_msgs::msg::JointTrajectory& trajectory);
    void computeAccelerations(trajectory_msgs::msg::JointTrajectory& trajectory);
    
    bool interpolatePoint(const trajectory_msgs::msg::JointTrajectoryPoint& p1,
                         const trajectory_msgs::msg::JointTrajectoryPoint& p2,
                         double ratio,
                         trajectory_msgs::msg::JointTrajectoryPoint& result);
    
    void updateTimeStamps(trajectory_msgs::msg::JointTrajectory& trajectory);
};

} // namespace motion_planner

#endif // TRAJECTORY_OPTIMIZER_HPP