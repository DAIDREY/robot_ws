#ifndef ROBOT_TASK_TASK_MANAGER_HPP
#define ROBOT_TASK_TASK_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_task/srv/grasp_object.hpp>
#include <robot_task/msg/object_pose.hpp>
#include <robot_task/msg/task_status.hpp>
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
    
    // 物体姿态回调 (从robot_visioner接收ObjectPose)
    void objectPoseCallback(const robot_task::msg::ObjectPose::SharedPtr msg);
    
    // 等待指定物体出现
    bool waitForObject(const std::string& object_name, robot_task::msg::ObjectPose& pose);
    
    // 计算抓取姿态
    geometry_msgs::msg::PoseStamped calculateGraspPose(const robot_task::msg::ObjectPose& object_pose);
    
    // 执行抓取流程
    bool executeGraspSequence(const geometry_msgs::msg::PoseStamped& grasp_pose);
    
    // 发布状态
    void publishStatus(uint8_t status, const std::string& message);
    
    // 控制夹爪
    // bool controlGripper(const std::string& command);
    
    // 等待运动完成 (简单时间延迟)
    bool waitForMotionComplete(double timeout_seconds = 5.0);

    void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void initializeStatusSubscription();
    bool waitForRobotReady(double timeout_seconds = 10.0);
    std::string parseRobotState(const std::string& status_message);
    void printRobotStatus();
    void executeGraspTask(std::string object_name);
    bool waitForObjectNonBlocking(const std::string& object_name, robot_task::msg::ObjectPose& pose, double timeout_seconds = 15.0);

    // ROS接口
    rclcpp::Service<robot_task::srv::GraspObject>::SharedPtr grasp_service_;
    rclcpp::Subscription<robot_task::msg::ObjectPose>::SharedPtr object_pose_sub_;  // 修改为ObjectPose
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<robot_task::msg::TaskStatus>::SharedPtr status_pub_;
    
    // 状态变量
    robot_task::msg::ObjectPose latest_object_pose_;  // 修改为ObjectPose
    bool object_detected_;
    std::string current_target_object_;
    bool task_active_;
    std::thread grasp_thread_;
    std::atomic<bool> grasp_thread_running_;
    std::string current_robot_state_;
    std::mutex robot_state_mutex_;
    std::condition_variable state_change_cv_;
    bool robot_ready_;
    
    double pre_grasp_height_;      // 预抓取高度偏移
    double grasp_height_offset_;   // 抓取高度偏移  
    double gripper_length_;        // 夹爪长度偏移 (保留)
    double lift_height_;           // 提升高度
    double place_offset_x_;        // 放置位置X偏移
    double place_offset_y_;        // 放置位置Y偏移
    double place_offset_z_;        // 放置位置Z偏移
    double wait_time_at_object_;   // 在物体位置的等待时间
    
    // 新增：初始位置参数
    double home_position_x_;       // 初始位置X
    double home_position_y_;       // 初始位置Y
    double home_position_z_;       // 初始位置Z
    double home_orientation_roll_; // 初始姿态Roll
    double home_orientation_pitch_;// 初始姿态Pitch
    double home_orientation_yaw_;  // 初始姿态Yaw
};

} // namespace robot_task

#endif // ROBOT_TASK_TASK_MANAGER_HPP