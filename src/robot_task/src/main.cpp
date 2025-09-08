#include "task_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建TaskManager节点
    auto node = std::make_shared<robot_task::TaskManager>();
    
    RCLCPP_INFO(node->get_logger(), "🚀 Robot Task 系统启动成功！");
    RCLCPP_INFO(node->get_logger(), "📋 使用方法:");
    RCLCPP_INFO(node->get_logger(), "   ros2 service call /grasp_object robot_task/srv/GraspObject \"{object_name: 'cup'}\"");
    RCLCPP_INFO(node->get_logger(), "📊 监控状态:");
    RCLCPP_INFO(node->get_logger(), "   ros2 topic echo /robot_task/status");
    
    try {
        // 运行节点
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "💥 节点运行异常: %s", e.what());
    }
    
    // 清理
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"), "👋 Robot Task 系统已关闭");
    
    return 0;
}