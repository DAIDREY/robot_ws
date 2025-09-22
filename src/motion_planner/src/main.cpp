#include <rclcpp/rclcpp.hpp>
#include "motion_planner.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<motion_planner::MotionPlannerNode>();
    
    RCLCPP_INFO(node->get_logger(), "运动规划节点启动完成");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "节点运行异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}