// include/robot_driver/serial_communication.hpp
#ifndef ROBOT_DRIVER_SERIAL_COMMUNICATION_HPP
#define ROBOT_DRIVER_SERIAL_COMMUNICATION_HPP

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_driver
{

// 3D位置结构
struct Position3D
{
    double x, y, z;
    
    Position3D(double x = 0.0, double y = 0.0, double z = 0.0) 
        : x(x), y(y), z(z) {}
};

// 姿态角度结构
struct Orientation
{
    double b0, b1, w;  // B0, B1, W姿态角 (度)
    
    Orientation(double b0 = 0.0, double b1 = 0.0, double w = 0.0) 
        : b0(b0), b1(b1), w(w) {}
};

// 关节角度结构
struct JointAngles
{
    double angles[6];  // 6个关节角度 (度)
    
    JointAngles() {
        for (int i = 0; i < 6; i++) angles[i] = 0.0;
    }
    
    JointAngles(double a0, double a1, double a2, double w0, double w1, double aw) {
        angles[0] = a0; angles[1] = a1; angles[2] = a2;
        angles[3] = w0; angles[4] = w1; angles[5] = aw;
    }
};

// 状态包结构 (11字节)
struct StatusPacket 
{
    uint8_t header;    // 帧头 0xCE
    uint8_t data[8];   // 8字节数据
    uint8_t flag;      // 标志位
    uint8_t tail;      // 帧尾 0xCF
} __attribute__((packed));

// 机器人状态结构
struct RobotStatus
{
    // 坐标信息
    double x, y, z;                    // 当前坐标 (mm)
    double joint_angles[6];            // 关节角度 (度)
    double orientation[3];             // 姿态角 B0, B1, W (度)
    
    // 工作台坐标
    double workspace_origin[3];        // 工作台原点 (mm)
    
    // PWM控制信息
    uint16_t pwm_value;               // PWM值
    
    // 时间戳
    std::chrono::system_clock::time_point timestamp;
    
    RobotStatus() {
        x = y = z = 0.0;
        for (int i = 0; i < 6; i++) joint_angles[i] = 0.0;
        for (int i = 0; i < 3; i++) orientation[i] = 0.0;
        for (int i = 0; i < 3; i++) workspace_origin[i] = 0.0;
        pwm_value = 1500;
        timestamp = std::chrono::system_clock::now();
    }
};

class SerialCommunication
{
public:
    using StatusCallback = std::function<void(const RobotStatus&)>;
    
    explicit SerialCommunication(rclcpp::Logger logger);
    ~SerialCommunication();
    
    // 连接管理
    bool connect(const std::string& port, unsigned int baudrate = 115200);
    void disconnect();
    bool isConnected() const;
    
    // 运动指令发送
    bool sendLinearMove(const Position3D& position, 
                       const Orientation& orientation = Orientation(),
                       double speed = 100.0, 
                       uint16_t pwm = 1500);
    
    bool sendJointMove(const JointAngles& joint_angles,
                      double speed = 30.0,
                      uint8_t buffer_control = 0,
                      uint16_t pwm = 1500);
    
    // 状态接收
    void setStatusCallback(StatusCallback callback);
    void startStatusReceiving();
    void stopStatusReceiving();
    RobotStatus getCurrentStatus() const;
    
private:
    // 内部方法
    bool sendCommand(const std::vector<uint8_t>& command);
    std::vector<uint8_t> floatToBytes(float value);
    
    void receiveLoop();
    bool findFrameHeader();
    void processStatusPacket(const StatusPacket& packet);
    void parseCoordinateAndAxis0(const StatusPacket& packet, RobotStatus& status);
    void parseAxisAngles2(const StatusPacket& packet, RobotStatus& status);
    void parseAxisAngles3AndWorkspace(const StatusPacket& packet, RobotStatus& status);
    void parsePWMAndAxis4(const StatusPacket& packet, RobotStatus& status);
    
    // 成员变量
    LibSerial::SerialPort serial_port_;
    rclcpp::Logger logger_;
    StatusCallback status_callback_;
    std::thread receive_thread_;
    std::atomic<bool> receiving_;
    std::atomic<bool> connected_;
    
    RobotStatus current_status_;
    mutable std::mutex status_mutex_;
};

} // namespace robot_driver

#endif // ROBOT_DRIVER_SERIAL_COMMUNICATION_HPP