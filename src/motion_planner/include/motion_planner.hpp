#ifndef MOTION_PLANNER_NODE_HPP
#define MOTION_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "motion_planner/srv/plan_cartesian_path.hpp"
#include "motion_planner/srv/plan_joint_path.hpp"
#include "motion_planner/srv/plan_grasp_sequence.hpp"
#include "trajectory_optimizer.hpp"

namespace motion_planner
{

class MotionPlannerNode : public rclcpp::Node
{
public:
    explicit MotionPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MotionPlannerNode() = default;

    // 核心规划接口
    bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                          double eef_step,
                          double jump_threshold,
                          trajectory_msgs::msg::JointTrajectory& trajectory,
                          double& fraction);

    bool planJointPath(const std::vector<double>& target_joint_positions,
                      trajectory_msgs::msg::JointTrajectory& trajectory);

    bool planGraspSequence(const geometry_msgs::msg::Pose& target_pose,
                          double approach_distance,
                          double retreat_distance,
                          trajectory_msgs::msg::JointTrajectory& approach_trajectory,
                          trajectory_msgs::msg::JointTrajectory& grasp_trajectory,
                          trajectory_msgs::msg::JointTrajectory& retreat_trajectory);

    // 逆运动学求解
    bool solveInverseKinematics(const geometry_msgs::msg::Pose& target_pose,
                               std::vector<double>& joint_solution);

    // 状态查询
    geometry_msgs::msg::Pose getCurrentPose();
    std::vector<double> getCurrentJointValues();
    bool isReachable(const geometry_msgs::msg::Pose& target_pose);

private:
    // MoveIt接口
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    
    // 轨迹优化器
    // std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer_;
    
    // ROS2发布者
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr current_pose_pub_;
    
    // ROS2订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    // ROS2服务
    rclcpp::Service<srv::PlanCartesianPath>::SharedPtr plan_cartesian_service_;
    rclcpp::Service<srv::PlanJointPath>::SharedPtr plan_joint_service_;
    rclcpp::Service<srv::PlanGraspSequence>::SharedPtr plan_grasp_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    
    // 配置参数
    std::string planning_group_;
    std::string end_effector_link_;
    std::string reference_frame_;
    double planning_time_;
    double velocity_scaling_factor_;
    double acceleration_scaling_factor_;
    int max_planning_attempts_;
    int default_max_trajectory_points_;
    
    // 状态变量
    sensor_msgs::msg::JointState current_joint_state_;
    std::mutex joint_state_mutex_;
    bool moveit_initialized_;
    
    // 初始化函数
    void initializeParameters();
    void initializeMoveIt();
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();
    
    // 回调函数
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    void planCartesianPathCallback(
        const std::shared_ptr<srv::PlanCartesianPath::Request> request,
        std::shared_ptr<srv::PlanCartesianPath::Response> response);
    
    void planJointPathCallback(
        const std::shared_ptr<srv::PlanJointPath::Request> request,
        std::shared_ptr<srv::PlanJointPath::Response> response);
    
    void planGraspSequenceCallback(
        const std::shared_ptr<srv::PlanGraspSequence::Request> request,
        std::shared_ptr<srv::PlanGraspSequence::Response> response);
    
    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // 辅助函数
    std::vector<geometry_msgs::msg::Pose> generateGraspWaypoints(
        const geometry_msgs::msg::Pose& target_pose,
        double approach_distance,
        double retreat_distance);
    
    bool validateJointPositions(const std::vector<double>& joint_positions);
    
    trajectory_msgs::msg::JointTrajectory createTrajectoryFromPlan(
        const moveit::planning_interface::MoveGroupInterface::Plan& plan);
    
    void publishCurrentPose();
    
    // 定时器
    rclcpp::TimerBase::SharedPtr pose_publish_timer_;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_NODE_HPP