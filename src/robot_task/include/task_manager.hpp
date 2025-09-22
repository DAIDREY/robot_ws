#ifndef ROBOT_TASK_TASK_MANAGER_HPP
#define ROBOT_TASK_TASK_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <robot_task/srv/grasp_object.hpp>
#include <robot_task/msg/object_pose.hpp>
#include <robot_task/msg/task_status.hpp>

#include <motion_planner/srv/plan_grasp_sequence.hpp>
#include <motion_planner/srv/plan_cartesian_path.hpp>
#include <motion_planner/srv/plan_joint_path.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <map>
#include <chrono>
#include <thread>

namespace robot_task
{

class TaskManager : public rclcpp::Node
{
public:
    TaskManager();
    ~TaskManager();

private:
    bool moveToHomePosition();
    geometry_msgs::msg::PoseStamped getHomePosition();

    // 服务回调：执行抓取任务
    void graspObjectCallback(
        const std::shared_ptr<robot_task::srv::GraspObject::Request> request,
        std::shared_ptr<robot_task::srv::GraspObject::Response> response);
    
    // 物体姿态回调
    void objectPoseCallback(const robot_task::msg::ObjectPose::SharedPtr msg);
    
    // 等待指定物体出现
    bool waitForObject(const std::string& object_name, robot_task::msg::ObjectPose& pose);
    
    // 计算抓取姿态
    geometry_msgs::msg::PoseStamped calculateGraspPose(const robot_task::msg::ObjectPose& object_pose);
    
    bool executeGraspSequenceWithMotionPlanner(const geometry_msgs::msg::PoseStamped& grasp_pose);
    
    bool executeGraspSequence(const geometry_msgs::msg::PoseStamped& grasp_pose);
    
    // 发布状态
    void publishStatus(uint8_t status, const std::string& message);
    
    // 等待运动完成
    bool waitForMotionComplete(double timeout_seconds = 5.0);

    void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void initializeStatusSubscription();
    bool waitForRobotReady(double timeout_seconds = 10.0);
    std::string parseRobotState(const std::string& status_message);
    void printRobotStatus();
    void executeGraspTask(std::string object_name);
    bool waitForObjectNonBlocking(const std::string& object_name, robot_task::msg::ObjectPose& pose, double timeout_seconds = 15.0);

    // 新增：motion_planner相关函数
    bool planGraspWithMotionPlanner(const geometry_msgs::msg::Pose& target_pose);
    bool sendTrajectoryToRobotDriver(const trajectory_msgs::msg::JointTrajectory& trajectory);
    bool waitForPlannerReady(double timeout_seconds = 5.0);

    // ROS接口
    rclcpp::Service<robot_task::srv::GraspObject>::SharedPtr grasp_service_;
    rclcpp::Subscription<robot_task::msg::ObjectPose>::SharedPtr object_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<robot_task::msg::TaskStatus>::SharedPtr status_pub_;
    
    // 新增：motion_planner客户端和发布者
    rclcpp::Client<motion_planner::srv::PlanGraspSequence>::SharedPtr motion_planner_client_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

    // 状态变量
    robot_task::msg::ObjectPose latest_object_pose_;
    bool object_detected_;
    std::string current_target_object_;
    bool task_active_;
    std::thread grasp_thread_;
    std::atomic<bool> grasp_thread_running_;
    std::string current_robot_state_;
    std::mutex robot_state_mutex_;
    std::condition_variable state_change_cv_;
    bool robot_ready_;
    
    // 配置参数
    double pre_grasp_height_;
    double grasp_height_offset_;
    double gripper_length_;
    double lift_height_;
    double place_offset_x_;
    double place_offset_y_;
    double place_offset_z_;
    double wait_time_at_object_;
    
    // 初始位置参数
    double home_position_x_;
    double home_position_y_;
    double home_position_z_;
    double home_orientation_roll_;
    double home_orientation_pitch_;
    double home_orientation_yaw_;
    
    // 新增：motion_planner配置
    bool use_motion_planner_;           // 是否使用motion_planner
    double approach_distance_;          // 接近距离
    double retreat_distance_;           // 后退距离
    int max_trajectory_points_;         // 最大轨迹点数
};

} // namespace robot_task

#endif // ROBOT_TASK_TASK_MANAGER_HPP