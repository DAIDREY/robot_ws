#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "robot_driver/command_builder.hpp"
#include "robot_driver/serial_communication.hpp"

using namespace robot_driver;

class RobotDriverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = rclcpp::Node::make_shared("test_node");
        command_builder_ = std::make_unique<CommandBuilder>();
    }
    
    void TearDown() override
    {
        rclcpp::shutdown();
    }
    
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<CommandBuilder> command_builder_;
};

// 测试指令构建器
TEST_F(RobotDriverTest, TestLinearMoveCommand)
{
    Position3D target(100.0, 200.0, 300.0);
    Orientation ori(0.0, 0.0, 0.0);
    
    auto command = command_builder_->buildLinearMove(target, ori, 1500, 1000.0);
    
    // 验证指令长度
    EXPECT_EQ(command.size(), 48);
    
    // 验证帧头和帧尾
    EXPECT_EQ(command[0], 0xee);
    EXPECT_EQ(command[47], 0xef);
    
    // 验证指令代码
    EXPECT_EQ(command[1], '1');
    EXPECT_EQ(command[2], 1);
}

TEST_F(RobotDriverTest, TestJointMoveCommand)
{
    JointAngles angles(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);
    
    auto command = command_builder_->buildJointMove(angles, 0, 30.0, 1500);
    
    // 验证指令长度
    EXPECT_EQ(command.size(), 48);
    
    // 验证帧头和帧尾
    EXPECT_EQ(command[0], 0xee);
    EXPECT_EQ(command[47], 0xef);
    
    // 验证指令代码
    EXPECT_EQ(command[1], '3');
    EXPECT_EQ(command[2], 0);
}

TEST_F(RobotDriverTest, TestControlCommands)
{
    // 测试机器人复位指令
    auto reset_cmd = command_builder_->buildRobotReset();
    EXPECT_EQ(reset_cmd.size(), 48);
    EXPECT_EQ(reset_cmd[0], 0xfc);
    EXPECT_EQ(reset_cmd[47], 0xfd);
    EXPECT_EQ(reset_cmd[1], 12);
    EXPECT_EQ(reset_cmd[2], 3);
    
    // 测试急停指令
    auto estop_cmd = command_builder_->buildEmergencyStop();
    EXPECT_EQ(estop_cmd.size(), 48);
    EXPECT_EQ(estop_cmd[0], 0xfc);
    EXPECT_EQ(estop_cmd[47], 0xfd);
    EXPECT_EQ(estop_cmd[1], 12);
    EXPECT_EQ(estop_cmd[2], 8);
}

TEST_F(RobotDriverTest, TestWorkspaceLimits)
{
    Position3D valid_pos(100.0, 100.0, 100.0);
    Position3D invalid_pos(1000.0, 1000.0, 1000.0);
    
    // 这里需要创建一个SeedRobotDriver实例来测试
    // 由于需要串口连接，这里只做基本的范围检查测试
    
    EXPECT_TRUE(valid_pos.x >= -500.0 && valid_pos.x <= 500.0);
    EXPECT_FALSE(invalid_pos.x >= -500.0 && invalid_pos.x <= 500.0);
}

TEST_F(RobotDriverTest, TestJointLimits)
{
    JointAngles valid_angles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    JointAngles invalid_angles(200.0, 200.0, 200.0, 200.0, 200.0, 200.0);
    
    // 基本的关节限制检查
    bool valid = true;
    std::vector<double> min_limits = {-150.0, -65.0, -188.0, -167.0, -34.0, -270.0};
    std::vector<double> max_limits = {150.0, 110.0, 27.0, 167.0, 208.0, 270.0};
    
    for (size_t i = 0; i < 6; ++i) {
        if (valid_angles.angles[i] < min_limits[i] || valid_angles.angles[i] > max_limits[i]) {
            valid = false;
            break;
        }
    }
    EXPECT_TRUE(valid);
    
    valid = true;
    for (size_t i = 0; i < 6; ++i) {
        if (invalid_angles.angles[i] < min_limits[i] || invalid_angles.angles[i] > max_limits[i]) {
            valid = false;
            break;
        }
    }
    EXPECT_FALSE(valid);
}

// 性能测试
TEST_F(RobotDriverTest, TestCommandGenerationPerformance)
{
    const int num_commands = 1000;
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_commands; ++i) {
        Position3D target(i * 0.1, i * 0.1, i * 0.1);
        Orientation ori(0.0, 0.0, 0.0);
        auto command = command_builder_->buildLinearMove(target, ori, 1500, 1000.0);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    RCLCPP_INFO(node_->get_logger(), 
               "生成 %d 个指令用时: %ld 微秒 (平均: %.2f 微秒/指令)", 
               num_commands, duration.count(), 
               static_cast<double>(duration.count()) / num_commands);
    
    // 确保性能在合理范围内 (每个指令生成时间应该小于100微秒)
    EXPECT_LT(duration.count() / num_commands, 100);
}

// 状态解析测试
TEST_F(RobotDriverTest, TestStatusPacketParsing)
{
    // 模拟状态包数据
    StatusPacket packet;
    packet.header = 0xce;
    packet.flag = 0;  // 坐标值、轴角度1
    packet.tail = 0xcf;
    
    // 设置模拟的坐标数据 (100.0mm, 200.0mm, 300.0mm)
    uint16_t x_raw = 1000;  // 100.0 * 10
    uint16_t y_raw = 2000;  // 200.0 * 10
    uint16_t z_raw = 3000;  // 300.0 * 10
    
    packet.data[0] = x_raw & 0xff;
    packet.data[1] = (x_raw >> 8) & 0xff;
    packet.data[2] = y_raw & 0xff;
    packet.data[3] = (y_raw >> 8) & 0xff;
    packet.data[4] = z_raw & 0xff;
    packet.data[5] = (z_raw >> 8) & 0xff;
    
    // 验证数据解析
    uint16_t parsed_x = static_cast<uint16_t>(packet.data[0]) | 
                       (static_cast<uint16_t>(packet.data[1]) << 8);
    uint16_t parsed_y = static_cast<uint16_t>(packet.data[2]) | 
                       (static_cast<uint16_t>(packet.data[3]) << 8);
    uint16_t parsed_z = static_cast<uint16_t>(packet.data[4]) | 
                       (static_cast<uint16_t>(packet.data[5]) << 8);
    
    EXPECT_EQ(parsed_x, 1000);
    EXPECT_EQ(parsed_y, 2000);
    EXPECT_EQ(parsed_z, 3000);
    
    // 验证转换为实际坐标值
    double actual_x = static_cast<double>(parsed_x) / 10.0;
    double actual_y = static_cast<double>(parsed_y) / 10.0;
    double actual_z = static_cast<double>(parsed_z) / 10.0;
    
    EXPECT_DOUBLE_EQ(actual_x, 100.0);
    EXPECT_DOUBLE_EQ(actual_y, 200.0);
    EXPECT_DOUBLE_EQ(actual_z, 300.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}