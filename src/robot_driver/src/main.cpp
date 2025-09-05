#include "seed_robot_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<robot_driver::SeedRobotDriver>();
        
        RCLCPP_INFO(node->get_logger(), "SEED机器人驱动器节点已启动");
        
        // 运行节点
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("seed_robot_driver"), 
                    "节点运行异常: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}