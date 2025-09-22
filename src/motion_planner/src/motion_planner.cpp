#include "motion_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace motion_planner
{

MotionPlannerNode::MotionPlannerNode(const rclcpp::NodeOptions& options)
    : Node("motion_planner_node", options), moveit_initialized_(false)
{
    RCLCPP_INFO(this->get_logger(), "运动规划节点启动中...");
    
    // 初始化参数
    initializeParameters();
    
    // 创建轨迹优化器
    // trajectory_optimizer_ = std::make_unique<TrajectoryOptimizer>(this->get_logger());
    
    // 初始化ROS2接口
    initializePublishers();
    initializeSubscribers();
    initializeServices();
    
    // 异步初始化MoveIt（等待其他节点启动）
    auto init_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            try {
                initializeMoveIt();
                RCLCPP_INFO(this->get_logger(), "✅ 运动规划节点初始化完成");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "等待MoveIt初始化: %s", e.what());
            }
        });
}

void MotionPlannerNode::initializeParameters()
{
    this->declare_parameter("planning_group", "arm");
    this->declare_parameter("end_effector_link", "link6");
    this->declare_parameter("reference_frame", "base_link");
    this->declare_parameter("planning_time", 5.0);
    this->declare_parameter("velocity_scaling_factor", 0.3);
    this->declare_parameter("acceleration_scaling_factor", 0.3);
    this->declare_parameter("max_planning_attempts", 10);
    this->declare_parameter("default_max_trajectory_points", 100);
    
    planning_group_ = this->get_parameter("planning_group").as_string();
    end_effector_link_ = this->get_parameter("end_effector_link").as_string();
    reference_frame_ = this->get_parameter("reference_frame").as_string();
    planning_time_ = this->get_parameter("planning_time").as_double();
    velocity_scaling_factor_ = this->get_parameter("velocity_scaling_factor").as_double();
    acceleration_scaling_factor_ = this->get_parameter("acceleration_scaling_factor").as_double();
    max_planning_attempts_ = this->get_parameter("max_planning_attempts").as_int();
    default_max_trajectory_points_ = this->get_parameter("default_max_trajectory_points").as_int();
}

void MotionPlannerNode::initializeMoveIt()
{
    if (moveit_initialized_) return;
    RCLCPP_INFO(this->get_logger(), "开始进行MoveIt初始化");
    // 创建MoveIt接口
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    
    // 配置MoveIt
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setNumPlanningAttempts(max_planning_attempts_);
    
    moveit_initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), "MoveIt初始化完成");
    RCLCPP_INFO(this->get_logger(), "  规划组: %s", planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), "  末端执行器: %s", end_effector_link_.c_str());
    RCLCPP_INFO(this->get_logger(), "  参考坐标系: %s", reference_frame_.c_str());
}

void MotionPlannerNode::initializePublishers()
{
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "planned_trajectory", 10);
    
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "current_pose", 10);
    
    // 定时发布当前位姿
    pose_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotionPlannerNode::publishCurrentPose, this));
}

void MotionPlannerNode::initializeSubscribers()
{
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&MotionPlannerNode::jointStateCallback, this, std::placeholders::_1));
}

void MotionPlannerNode::initializeServices()
{
    plan_cartesian_service_ = this->create_service<srv::PlanCartesianPath>(
        "plan_cartesian_path",
        std::bind(&MotionPlannerNode::planCartesianPathCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    plan_joint_service_ = this->create_service<srv::PlanJointPath>(
        "plan_joint_path",
        std::bind(&MotionPlannerNode::planJointPathCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    plan_grasp_service_ = this->create_service<srv::PlanGraspSequence>(
        "plan_grasp_sequence",
        std::bind(&MotionPlannerNode::planGraspSequenceCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_planner",
        std::bind(&MotionPlannerNode::resetCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
}

bool MotionPlannerNode::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                         double eef_step,
                                         double jump_threshold,
                                         trajectory_msgs::msg::JointTrajectory& trajectory,
                                         double& fraction)
{
    if (!moveit_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt未初始化");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "规划笛卡尔路径，路径点数: %zu", waypoints.size());
    
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, robot_trajectory);
    
    if (fraction > 0.0) {
        trajectory = robot_trajectory.joint_trajectory;
        
        // 优化轨迹点数
        // if (trajectory.points.size() > default_max_trajectory_points_) {
        //     trajectory_optimizer_->optimizeTrajectory(trajectory, default_max_trajectory_points_);
        // }
        
        RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划成功，完成度: %.1f%%，轨迹点数: %zu", 
                   fraction * 100, trajectory.points.size());
        return true;
    }
    
    RCLCPP_WARN(this->get_logger(), "笛卡尔路径规划失败");
    return false;
}

bool MotionPlannerNode::planJointPath(const std::vector<double>& target_joint_positions,
                                     trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (!moveit_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt未初始化");
        return false;
    }
    
    if (!validateJointPositions(target_joint_positions)) {
        RCLCPP_ERROR(this->get_logger(), "关节位置验证失败");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "规划关节路径");
    
    move_group_->setJointValueTarget(target_joint_positions);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        trajectory = plan.trajectory_.joint_trajectory;
        
        // 优化轨迹点数
        // if (trajectory.points.size() > default_max_trajectory_points_) {
        //     trajectory_optimizer_->optimizeTrajectory(trajectory, default_max_trajectory_points_);
        // }
        
        RCLCPP_INFO(this->get_logger(), "关节路径规划成功，轨迹点数: %zu", trajectory.points.size());
        return true;
    }
    
    RCLCPP_ERROR(this->get_logger(), "关节路径规划失败");
    return false;
}

bool MotionPlannerNode::planGraspSequence(const geometry_msgs::msg::Pose& target_pose,
                                         double approach_distance,
                                         double retreat_distance,
                                         trajectory_msgs::msg::JointTrajectory& approach_trajectory,
                                         trajectory_msgs::msg::JointTrajectory& grasp_trajectory,
                                         trajectory_msgs::msg::JointTrajectory& retreat_trajectory)
{
    if (!moveit_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt未初始化");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "规划抓取序列");
    
    // 生成抓取路径点
    auto waypoints = generateGraspWaypoints(target_pose, approach_distance, retreat_distance);
    
    // 规划完整路径
    trajectory_msgs::msg::JointTrajectory full_trajectory;
    double fraction;
    
    if (!planCartesianPath(waypoints, 0.01, 0.0, full_trajectory, fraction)) {
        RCLCPP_ERROR(this->get_logger(), "抓取路径规划失败");
        return false;
    }
    
    if (fraction < 0.8) {
        RCLCPP_WARN(this->get_logger(), "抓取路径不完整: %.1f%%", fraction * 100);
    }
    
    // 分割轨迹
    size_t total_points = full_trajectory.points.size();
    size_t approach_points = total_points / 3;
    size_t grasp_points = total_points / 3;
    size_t retreat_points = total_points - approach_points - grasp_points;
    
    // 创建分段轨迹
    approach_trajectory = full_trajectory;
    approach_trajectory.points.resize(approach_points);
    
    grasp_trajectory = full_trajectory;
    grasp_trajectory.points.assign(
        full_trajectory.points.begin() + approach_points,
        full_trajectory.points.begin() + approach_points + grasp_points);
    
    retreat_trajectory = full_trajectory;
    retreat_trajectory.points.assign(
        full_trajectory.points.begin() + approach_points + grasp_points,
        full_trajectory.points.end());
    
    RCLCPP_INFO(this->get_logger(), "抓取序列规划成功");
    RCLCPP_INFO(this->get_logger(), "  接近轨迹: %zu点", approach_trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "  抓取轨迹: %zu点", grasp_trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "  后退轨迹: %zu点", retreat_trajectory.points.size());
    
    return true;
}

bool MotionPlannerNode::solveInverseKinematics(const geometry_msgs::msg::Pose& target_pose,
                                              std::vector<double>& joint_solution)
{
    if (!moveit_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt未初始化");
        return false;
    }
    
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    if (!current_state) {
        RCLCPP_ERROR(this->get_logger(), "无法获取当前机器人状态");
        return false;
    }
    
    const moveit::core::JointModelGroup* joint_model_group = 
        current_state->getJointModelGroup(planning_group_);
    
    bool found_ik = current_state->setFromIK(joint_model_group, target_pose, 10);
    
    if (found_ik) {
        current_state->copyJointGroupPositions(joint_model_group, joint_solution);
        RCLCPP_DEBUG(this->get_logger(), "逆运动学求解成功");
        return true;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "逆运动学无解");
    return false;
}

geometry_msgs::msg::Pose MotionPlannerNode::getCurrentPose()
{
    if (moveit_initialized_) {
        return move_group_->getCurrentPose().pose;
    }
    return geometry_msgs::msg::Pose();
}

std::vector<double> MotionPlannerNode::getCurrentJointValues()
{
    if (moveit_initialized_) {
        return move_group_->getCurrentJointValues();
    }
    return std::vector<double>();
}

bool MotionPlannerNode::isReachable(const geometry_msgs::msg::Pose& target_pose)
{
    std::vector<double> joint_solution;
    return solveInverseKinematics(target_pose, joint_solution);
}

void MotionPlannerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    current_joint_state_ = *msg;
}

void MotionPlannerNode::planCartesianPathCallback(
    const std::shared_ptr<srv::PlanCartesianPath::Request> request,
    std::shared_ptr<srv::PlanCartesianPath::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "接收笛卡尔路径规划请求");
    
    trajectory_msgs::msg::JointTrajectory trajectory;
    double fraction;
    
    int max_points = (request->max_trajectory_points > 0) ? 
                     request->max_trajectory_points : default_max_trajectory_points_;
    
    bool success = planCartesianPath(request->waypoints, request->eef_step, 
                                   request->jump_threshold, trajectory, fraction);
    
    // if (success && trajectory.points.size() > max_points) {
    //     trajectory_optimizer_->optimizeTrajectory(trajectory, max_points);
    // }
    
    response->success = success;
    response->trajectory = trajectory;
    response->fraction = fraction;
    response->num_points = static_cast<int32_t>(trajectory.points.size());
    response->message = success ? "规划成功" : "规划失败";
    
    if (success) {
        trajectory_pub_->publish(trajectory);
    }
}

void MotionPlannerNode::planJointPathCallback(
    const std::shared_ptr<srv::PlanJointPath::Request> request,
    std::shared_ptr<srv::PlanJointPath::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "接收关节路径规划请求");
    
    // 临时设置速度和加速度缩放因子
    double old_vel = velocity_scaling_factor_;
    double old_acc = acceleration_scaling_factor_;
    
    if (request->velocity_scaling_factor > 0) {
        move_group_->setMaxVelocityScalingFactor(request->velocity_scaling_factor);
    }
    if (request->acceleration_scaling_factor > 0) {
        move_group_->setMaxAccelerationScalingFactor(request->acceleration_scaling_factor);
    }
    
    trajectory_msgs::msg::JointTrajectory trajectory;
    bool success = planJointPath(request->target_joint_positions, trajectory);
    
    int max_points = (request->max_trajectory_points > 0) ? 
                     request->max_trajectory_points : default_max_trajectory_points_;
    
    // if (success && trajectory.points.size() > max_points) {
    //     trajectory_optimizer_->optimizeTrajectory(trajectory, max_points);
    // }
    
    response->success = success;
    response->trajectory = trajectory;
    response->num_points = static_cast<int32_t>(trajectory.points.size());
    response->message = success ? "规划成功" : "规划失败";
    
    // 恢复原始设置
    move_group_->setMaxVelocityScalingFactor(old_vel);
    move_group_->setMaxAccelerationScalingFactor(old_acc);
    
    if (success) {
        trajectory_pub_->publish(trajectory);
    }
}

void MotionPlannerNode::planGraspSequenceCallback(
    const std::shared_ptr<srv::PlanGraspSequence::Request> request,
    std::shared_ptr<srv::PlanGraspSequence::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "接收抓取序列规划请求");
    
    trajectory_msgs::msg::JointTrajectory approach_traj, grasp_traj, retreat_traj;
    
    bool success = planGraspSequence(request->target_pose, request->approach_distance,
                                   request->retreat_distance, approach_traj, 
                                   grasp_traj, retreat_traj);
    
    int max_points = (request->max_trajectory_points > 0) ? 
                     request->max_trajectory_points : default_max_trajectory_points_;
    
    // 优化各段轨迹
    // if (success) {
    //     int points_per_segment = max_points / 3;
    //     if (approach_traj.points.size() > points_per_segment) {
    //         trajectory_optimizer_->optimizeTrajectory(approach_traj, points_per_segment);
    //     }
    //     if (grasp_traj.points.size() > points_per_segment) {
    //         trajectory_optimizer_->optimizeTrajectory(grasp_traj, points_per_segment);
    //     }
    //     if (retreat_traj.points.size() > points_per_segment) {
    //         trajectory_optimizer_->optimizeTrajectory(retreat_traj, points_per_segment);
    //     }
    // }
    
    response->success = success;
    response->approach_trajectory = approach_traj;
    response->grasp_trajectory = grasp_traj;
    response->retreat_trajectory = retreat_traj;
    response->total_points = static_cast<int32_t>(
        approach_traj.points.size() + grasp_traj.points.size() + retreat_traj.points.size());
    response->message = success ? "抓取序列规划成功" : "抓取序列规划失败";
}

void MotionPlannerNode::resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "重置运动规划器");
    
    try {
        if (moveit_initialized_) {
            move_group_->clearPoseTargets();
            move_group_->stop();
        }
        response->success = true;
        response->message = "运动规划器重置成功";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("重置失败: ") + e.what();
    }
}

std::vector<geometry_msgs::msg::Pose> MotionPlannerNode::generateGraspWaypoints(
    const geometry_msgs::msg::Pose& target_pose,
    double approach_distance,
    double retreat_distance)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    // 当前位置
    if (moveit_initialized_) {
        waypoints.push_back(getCurrentPose());
    }
    
    // 接近点
    geometry_msgs::msg::Pose approach_pose = target_pose;
    approach_pose.position.z += approach_distance;
    waypoints.push_back(approach_pose);
    
    // 抓取点
    waypoints.push_back(target_pose);
    
    // 后退点
    geometry_msgs::msg::Pose retreat_pose = target_pose;
    retreat_pose.position.z += retreat_distance;
    waypoints.push_back(retreat_pose);
    
    return waypoints;
}

bool MotionPlannerNode::validateJointPositions(const std::vector<double>& joint_positions)
{
    if (joint_positions.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "关节位置数量错误: %zu (期望6个)", joint_positions.size());
        return false;
    }
    
    // 这里可以添加关节限制检查
    // TODO: 从参数或URDF获取关节限制
    
    return true;
}

void MotionPlannerNode::publishCurrentPose()
{
    if (moveit_initialized_) {
        auto pose = getCurrentPose();
        current_pose_pub_->publish(pose);
    }
}

} // namespace motion_planner