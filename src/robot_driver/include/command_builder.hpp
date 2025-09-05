// include/robot_driver/command_builder.hpp
#ifndef ROBOT_DRIVER_COMMAND_BUILDER_HPP
#define ROBOT_DRIVER_COMMAND_BUILDER_HPP

#include <vector>
#include <cstdint>
#include <cstddef>  // 添加size_t定义

namespace robot_driver
{

struct Position3D
{
    double x, y, z;
    Position3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct Orientation
{
    double b0, b1, w;
    Orientation(double b0 = 0, double b1 = 0, double w = 0) : b0(b0), b1(b1), w(w) {}
};

struct JointAngles
{
    double angles[6];
    JointAngles() { for(int i = 0; i < 6; i++) angles[i] = 0.0; }
    JointAngles(double a0, double a1, double a2, double w0, double w1, double aw)
    {
        angles[0] = a0; angles[1] = a1; angles[2] = a2;
        angles[3] = w0; angles[4] = w1; angles[5] = aw;
    }
};

class CommandBuilder
{
public:
    CommandBuilder();
    
    // 运动指令 (帧头 0xee, 帧尾 0xef)
    std::vector<uint8_t> buildLinearMove(
        const Position3D& target,
        const Orientation& orientation,
        uint16_t pwm = 1500,
        double speed = 1000.0
    );
    
    std::vector<uint8_t> buildLinearMoveWithAccel(
        const Position3D& target,
        const Orientation& orientation,
        uint8_t accel_mode,           // 0: 单加减速, 1: 先加速后减速, 2: 初速度到末速度
        double initial_speed = 0.0,
        double acceleration = 1000.0,
        double final_speed = 1000.0,
        uint16_t pwm = 1500
    );
    
    std::vector<uint8_t> buildJointMove(
        const JointAngles& joint_angles,
        uint8_t buffer_control = 0,   // 0-9: 缓冲控制
        double speed = 30.0,          // 度/秒
        uint16_t pwm = 1500
    );
    
    std::vector<uint8_t> buildArcMove(
        const Position3D& intermediate_point,
        double radius,
        const Orientation& orientation,
        bool is_first_command,        // true: G300, false: G301
        double speed = 30.0,
        uint16_t pwm = 1500
    );
    
    std::vector<uint8_t> buildSingleAxisMove(
        uint8_t axis_number,          // 0-7
        double angle_increment,       // 增量角度
        uint8_t buffer_control = 0,
        double speed = 30.0,
        uint16_t pwm = 1500
    );
    
    std::vector<uint8_t> buildDelay(uint32_t delay_ms);
    
    // 控制指令 (帧头 0xfc, 帧尾 0xfd)
    std::vector<uint8_t> buildRobotReset();
    std::vector<uint8_t> buildConnectorReset();
    std::vector<uint8_t> buildSetWorkspaceOrigin(const Position3D& origin);
    std::vector<uint8_t> buildMCode(uint32_t m_code);
    std::vector<uint8_t> buildRunProgram(uint8_t program_number, uint8_t loop_count = 1, uint16_t pause_time = 0);
    std::vector<uint8_t> buildPauseMotion();
    std::vector<uint8_t> buildFixedPosture(uint8_t posture_number);  // 0-3
    std::vector<uint8_t> buildSetL3Value(double l3_value);
    std::vector<uint8_t> buildJoystickControl(uint8_t control_type, uint8_t axis_value, uint8_t speed_mode = 1);
    std::vector<uint8_t> buildStructureMode(uint8_t mode);  // 0: 机器视觉模式, 9: 默认模式
    std::vector<uint8_t> buildStatusMode(uint8_t mode);     // 0: 仅文件信息, 1: +XYZ, 2: 全部
    std::vector<uint8_t> buildExternalAxisReset(uint8_t axis_number);  // 0: 轴1, 1: 轴2
    std::vector<uint8_t> buildEmergencyStop();
    std::vector<uint8_t> buildEmergencyRelease();
    std::vector<uint8_t> buildAutoCalibration();
    
private:
    void setFloatValue(std::vector<uint8_t>& command, size_t offset, float value);
    void setUint16Value(std::vector<uint8_t>& command, size_t offset, uint16_t value);
    void setUint32Value(std::vector<uint8_t>& command, size_t offset, uint32_t value);
    
    std::vector<uint8_t> createMotionCommand(char command_a, uint8_t command_b);
    std::vector<uint8_t> createControlCommand(uint8_t command_a, uint8_t command_b);
};

} // namespace robot_driver

#endif // ROBOT_DRIVER_COMMAND_BUILDER_HPP