// src/command_builder.cpp
#include "command_builder.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace robot_driver
{

CommandBuilder::CommandBuilder()
{
    // 构造函数 - 无需特殊初始化
}

std::vector<uint8_t> CommandBuilder::buildLinearMove(
    const Position3D& target,
    const Orientation& orientation,
    uint16_t pwm,
    double speed)
{
    // 创建G1直线插补指令
    auto command = createMotionCommand('1', 1);
    
    // 设置目标坐标 (mm)
    setFloatValue(command, 3, static_cast<float>(target.x));
    setFloatValue(command, 7, static_cast<float>(target.y));
    setFloatValue(command, 11, static_cast<float>(target.z));
    
    // 设置姿态角 (度)
    setFloatValue(command, 15, static_cast<float>(orientation.b0));
    setFloatValue(command, 19, static_cast<float>(orientation.b1));
    setFloatValue(command, 23, static_cast<float>(orientation.w));
    
    // 设置PWM值 (1us/步)
    setFloatValue(command, 27, static_cast<float>(pwm));
    
    // 外部轴角度值 (设为0)
    setFloatValue(command, 31, 0.0f);  // E0外部轴
    setFloatValue(command, 35, 0.0f);  // E1外部轴
    
    // 设置速度 (mm/分钟)
    setFloatValue(command, 43, static_cast<float>(speed));
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildLinearMoveWithAccel(
    const Position3D& target,
    const Orientation& orientation,
    uint8_t accel_mode,
    double initial_speed,
    double acceleration,
    double final_speed,
    uint16_t pwm)
{
    // 创建G220/G222/G230带加减速直线插补指令
    auto command = createMotionCommand('2', accel_mode);
    
    // 设置目标坐标 (mm)
    setFloatValue(command, 3, static_cast<float>(target.x));
    setFloatValue(command, 7, static_cast<float>(target.y));
    setFloatValue(command, 11, static_cast<float>(target.z));
    
    // 设置姿态角 (度)
    setFloatValue(command, 15, static_cast<float>(orientation.b0));
    setFloatValue(command, 19, static_cast<float>(orientation.b1));
    setFloatValue(command, 23, static_cast<float>(orientation.w));
    
    // 设置PWM值
    setFloatValue(command, 27, static_cast<float>(pwm));
    
    // 外部轴角度值 (设为0)
    setFloatValue(command, 31, 0.0f);  // E0
    setFloatValue(command, 35, 0.0f);  // E1
    
    // 设置运动参数
    switch (accel_mode) {
        case 0: // 单加速或单减速
            setFloatValue(command, 39, static_cast<float>(initial_speed));  // Speed0初速度
            setFloatValue(command, 43, static_cast<float>(acceleration));   // 加速度a
            setFloatValue(command, 47, static_cast<float>(final_speed));    // Speed1末速度
            break;
        case 1: // 先加速后减速
            setFloatValue(command, 39, static_cast<float>(initial_speed));  // Speed0初速度(通常为0)
            setFloatValue(command, 43, static_cast<float>(acceleration));   // 加速度a
            setFloatValue(command, 47, static_cast<float>(final_speed));    // Speed1末速度
            break;
        case 2: // 从初速度加速到末速度
            setFloatValue(command, 39, static_cast<float>(initial_speed));  // Speed0初速度
            setFloatValue(command, 43, static_cast<float>(acceleration));   // 加速度a
            setFloatValue(command, 47, static_cast<float>(final_speed));    // Speed1末速度
            break;
        default:
            // 默认模式1
            setFloatValue(command, 39, 0.0f);                              // 从0开始
            setFloatValue(command, 43, static_cast<float>(acceleration));   // 加速度
            setFloatValue(command, 47, static_cast<float>(final_speed));    // 目标速度
            break;
    }
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildJointMove(
    const JointAngles& joint_angles,
    uint8_t buffer_control,
    double speed,
    uint16_t pwm)
{
    // 创建G330轴角度插补指令
    auto command = createMotionCommand('3', buffer_control);
    
    // 设置6个关节角度 (度)
    setFloatValue(command, 3, static_cast<float>(joint_angles.angles[0]));   // a0 (关节1)
    setFloatValue(command, 7, static_cast<float>(joint_angles.angles[1]));   // a1 (关节2)
    setFloatValue(command, 11, static_cast<float>(joint_angles.angles[2]));  // a2 (关节3)
    setFloatValue(command, 15, static_cast<float>(joint_angles.angles[3]));  // w0 (关节4)
    setFloatValue(command, 19, static_cast<float>(joint_angles.angles[4]));  // w1 (关节5)
    setFloatValue(command, 23, static_cast<float>(joint_angles.angles[5]));  // aw (关节6)
    
    // 设置PWM值 (1us/步)
    setFloatValue(command, 27, static_cast<float>(pwm));
    
    // 外部轴角度值 (设为0)
    setFloatValue(command, 31, 0.0f);  // E0
    setFloatValue(command, 35, 0.0f);  // E1
    
    // 设置转速 (度/秒，以变化量最大的轴角度计算速度)
    setFloatValue(command, 43, static_cast<float>(speed));
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildArcMove(
    const Position3D& intermediate_point,
    double radius,
    const Orientation& orientation,
    bool is_first_command,
    double speed,
    uint16_t pwm)
{
    // 创建G300/G301三维圆弧插补指令
    uint8_t command_b = is_first_command ? 0 : 1;  // 0: G300第一条, 1: G301第二条
    auto command = createMotionCommand('4', command_b);
    
    if (is_first_command) {
        // 第一条指令 (G300)：中间点坐标 + 半径 + 姿态角 + 初始速度
        setFloatValue(command, 3, static_cast<float>(intermediate_point.x));
        setFloatValue(command, 7, static_cast<float>(intermediate_point.y));
        setFloatValue(command, 11, static_cast<float>(intermediate_point.z));
        
        // 设置姿态角
        setFloatValue(command, 15, static_cast<float>(orientation.b0));
        setFloatValue(command, 19, static_cast<float>(orientation.b1));
        setFloatValue(command, 23, static_cast<float>(orientation.w));
        
        // 设置PWM值
        setFloatValue(command, 27, static_cast<float>(pwm));
        
        // 设置半径 (mm)
        setFloatValue(command, 39, static_cast<float>(radius));
        
        // 设置初始速度 (度/秒)
        setFloatValue(command, 43, static_cast<float>(speed));
    } else {
        // 第二条指令 (G301)：结束点坐标 + 姿态角 + 末速度
        setFloatValue(command, 3, static_cast<float>(intermediate_point.x));  // 实际为结束点
        setFloatValue(command, 7, static_cast<float>(intermediate_point.y));
        setFloatValue(command, 11, static_cast<float>(intermediate_point.z));
        
        // 设置姿态角
        setFloatValue(command, 15, static_cast<float>(orientation.b0));
        setFloatValue(command, 19, static_cast<float>(orientation.b1));
        setFloatValue(command, 23, static_cast<float>(orientation.w));
        
        // 设置PWM值
        setFloatValue(command, 27, static_cast<float>(pwm));
        
        // 设置末速度 (度/秒)
        setFloatValue(command, 43, static_cast<float>(speed));
    }
    
    // 外部轴角度值 (设为0)
    setFloatValue(command, 31, 0.0f);  // E0
    setFloatValue(command, 35, 0.0f);  // E1
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildSingleAxisMove(
    uint8_t axis_number,
    double angle_increment,
    uint8_t buffer_control,
    double speed,
    uint16_t pwm)
{
    // 创建G336单轴角度插补指令
    auto command = createMotionCommand('5', buffer_control);
    
    // 设置轴号和角度增量
    setFloatValue(command, 3, static_cast<float>(axis_number));      // 轴号 (0-7)
    setFloatValue(command, 7, static_cast<float>(angle_increment));  // 角度增量 (度)
    
    // 其他浮点数设为0
    setFloatValue(command, 11, 0.0f);
    setFloatValue(command, 15, 0.0f);
    setFloatValue(command, 19, 0.0f);
    setFloatValue(command, 23, 0.0f);
    
    // 设置PWM值 (1us/步)
    setFloatValue(command, 27, static_cast<float>(pwm));
    
    // 外部轴角度值 (设为0)
    setFloatValue(command, 31, 0.0f);  // E0
    setFloatValue(command, 35, 0.0f);  // E1
    
    // 设置速度 (度/秒)
    setFloatValue(command, 43, static_cast<float>(speed));
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildDelay(uint32_t delay_ms)
{
    // 创建G4延时指令
    auto command = createMotionCommand('6', 0);
    
    // 设置延时值 (毫秒)
    setFloatValue(command, 3, static_cast<float>(delay_ms));
    
    // 其他值全部设为0
    for (size_t i = 7; i < 47; i += 4) {
        setFloatValue(command, i, 0.0f);
    }
    
    return command;
}

// 控制指令实现
std::vector<uint8_t> CommandBuilder::buildRobotReset()
{
    // 机械臂复位控制指令
    auto command = createControlCommand(12, 3);
    return command;
}

std::vector<uint8_t> CommandBuilder::buildConnectorReset()
{
    // 上位机连接器复位控制指令
    auto command = createControlCommand(12, 6);
    return command;
}

std::vector<uint8_t> CommandBuilder::buildSetWorkspaceOrigin(const Position3D& origin)
{
    // 设定工作台坐标原点控制指令 (G54)
    auto command = createControlCommand(20, 1);
    
    // 设置工作台坐标原点 (mm)
    setFloatValue(command, 3, static_cast<float>(origin.x));   // X0
    setFloatValue(command, 7, static_cast<float>(origin.y));   // Y0
    setFloatValue(command, 11, static_cast<float>(origin.z));  // Z0
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildMCode(uint32_t m_code)
{
    // M代码指令
    auto command = createControlCommand(22, 0);
    
    // 设置M代码值
    setUint32Value(command, 3, m_code);
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildRunProgram(uint8_t program_number, uint8_t loop_count, uint16_t pause_time)
{
    // 运行SD卡里存储的G代码程序控制指令
    auto command = createControlCommand(30, 2);
    
    command[3] = program_number;   // 程序号 (0-255)
    command[4] = loop_count;       // 循环次数 (1-255)
    
    // 暂停时间 (毫秒，16位整数)
    command[5] = static_cast<uint8_t>(pause_time >> 8);   // 高8位
    command[6] = static_cast<uint8_t>(pause_time & 0xff); // 低8位
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildPauseMotion()
{
    // 暂停运动控制指令 (此功能有问题暂不使用)
    auto command = createControlCommand(30, 3);
    command[3] = 2;  // 固定值
    return command;
}

std::vector<uint8_t> CommandBuilder::buildFixedPosture(uint8_t posture_number)
{
    // 机械臂的4个固定姿态控制指令
    auto command = createControlCommand(30, 5);
    command[3] = posture_number;  // 姿态号 (0-3)
    return command;
}

std::vector<uint8_t> CommandBuilder::buildSetL3Value(double l3_value)
{
    // 即时设置机器人L3值
    auto command = createControlCommand(30, 30);
    
    // 设置L3值 (mm)
    setFloatValue(command, 3, static_cast<float>(l3_value));
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildJoystickControl(uint8_t control_type, uint8_t axis_value, uint8_t speed_mode)
{
    // 模拟摇杆控制指令
    auto command = createControlCommand(30, 7);
    
    command[3] = control_type;  // 控制类型
    command[4] = axis_value;    // 轴值 (0-255)
    
    // 设置速度模式 (仅适用于2023年3月以后出厂的机械臂)
    command[7] = speed_mode;    // 1: 慢速, 2: 快速
    
    /*
     * 控制类型说明:
     * 0-5: 控制机械臂的1-6轴转动
     * 8: 控制夹爪动作
     * 100: 控制机械臂做直线运动 (需要配合command[5], command[6]设置Y、Z轴)
     * 200: 退出模拟摇杆状态
     * 
     * 轴值说明:
     * 108-148: 轴不动作
     * <108: 角度减小方向运转，值越小越快
     * >148: 角度增加方向运转，值越大越快
     */
    
    return command;
}

std::vector<uint8_t> CommandBuilder::buildStructureMode(uint8_t mode)
{
    // 结构模式控制指令
    auto command = createControlCommand(30, 9);
    command[3] = mode;  // 0: 机器视觉智能抓取模式, 9: 默认模式1
    return command;
}

std::vector<uint8_t> CommandBuilder::buildStatusMode(uint8_t mode)
{
    // 机械臂发送状态信息控制指令
    auto command = createControlCommand(30, 10);
    command[3] = mode;  // 0: 仅文件信息, 1: +XYZ值, 2: 全部状态信息
    return command;
}

std::vector<uint8_t> CommandBuilder::buildExternalAxisReset(uint8_t axis_number)
{
    // 外部轴复位指令
    auto command = createControlCommand(30, 12);
    command[3] = axis_number;  // 0: 复位外部轴1, 1: 复位外部轴2
    return command;
}

std::vector<uint8_t> CommandBuilder::buildEmergencyStop()
{
    // 急停指令
    auto command = createControlCommand(12, 8);
    command[3] = 0;  // 固定值
    return command;
}

std::vector<uint8_t> CommandBuilder::buildEmergencyRelease()
{
    // 解除急停指令
    auto command = createControlCommand(12, 9);
    command[3] = 0;  // 固定值
    return command;
}

std::vector<uint8_t> CommandBuilder::buildAutoCalibration()
{
    // 机械臂自动校准指令
    auto command = createControlCommand(12, 5);
    return command;
}

// 私有辅助函数实现
void CommandBuilder::setFloatValue(std::vector<uint8_t>& command, size_t offset, float value)
{
    if (offset + sizeof(float) <= command.size()) {
        // 使用memcpy确保字节序正确
        memcpy(&command[offset], &value, sizeof(float));
    }
}

void CommandBuilder::setUint16Value(std::vector<uint8_t>& command, size_t offset, uint16_t value)
{
    if (offset + sizeof(uint16_t) <= command.size()) {
        // 小端序存储
        command[offset] = static_cast<uint8_t>(value & 0xff);
        command[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
    }
}

void CommandBuilder::setUint32Value(std::vector<uint8_t>& command, size_t offset, uint32_t value)
{
    if (offset + sizeof(uint32_t) <= command.size()) {
        // 小端序存储
        command[offset] = static_cast<uint8_t>(value & 0xff);
        command[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
        command[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xff);
        command[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xff);
    }
}

std::vector<uint8_t> CommandBuilder::createMotionCommand(char command_a, uint8_t command_b)
{
    // 创建48字节运动指令
    std::vector<uint8_t> command(48);
    
    // 设置运动指令帧头和帧尾
    command[0] = 0xee;  // 运动指令帧头
    command[47] = 0xef; // 运动指令帧尾
    
    // 设置指令代码
    command[1] = static_cast<uint8_t>(command_a);  // 指令a (字符值)
    command[2] = command_b;                        // 指令b
    
    // 初始化其他字节为0
    for (size_t i = 3; i < 47; ++i) {
        command[i] = 0;
    }
    
    return command;
}

std::vector<uint8_t> CommandBuilder::createControlCommand(uint8_t command_a, uint8_t command_b)
{
    // 创建48字节控制指令
    std::vector<uint8_t> command(48);
    
    // 设置控制指令帧头和帧尾
    command[0] = 0xfc;  // 控制指令帧头
    command[47] = 0xfd; // 控制指令帧尾
    
    // 设置指令代码
    command[1] = command_a;  // 指令a
    command[2] = command_b;  // 指令b
    
    // 初始化其他字节为0
    for (size_t i = 3; i < 47; ++i) {
        command[i] = 0;
    }
    
    return command;
}

} // namespace robot_driver