// include/robot_driver/serial_communication.hpp
#ifndef ROBOT_DRIVER_SERIAL_COMMUNICATION_HPP
#define ROBOT_DRIVER_SERIAL_COMMUNICATION_HPP

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_driver
{

struct StatusPacket 
{
    uint8_t header;
    uint8_t data[8];
    uint8_t flag;
    uint8_t tail;
} __attribute__((packed));

struct RobotStatus
{
    // 坐标信息
    double x, y, z;                    // 当前坐标 (mm)
    double joint_angles[6];            // 关节角度 (度)
    double orientation[3];             // 姿态角 B0, B1, W
    
    // 工作台坐标
    double workspace_origin[3];        // 工作台原点
    
    // PWM和控制信息
    uint16_t pwm_value;               // PWM值
    
    // 文件运行状态
    uint32_t file_position;           // 文件运行位置
    uint16_t buffer_index;            // 缓冲区索引
    uint8_t structure_mode;           // 结构模式
    uint8_t error_code;               // 错误代码
    
    // 时间戳
    std::chrono::system_clock::time_point timestamp;
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
    
    // 数据发送
    bool sendCommand(const std::vector<uint8_t>& command);
    bool sendBytes(const uint8_t* data, size_t length);
    
    // 状态接收
    void setStatusCallback(StatusCallback callback);
    void startStatusReceiving();
    void stopStatusReceiving();
    
    // 控制指令
    bool sendRobotReset();
    bool sendConnectorReset();
    bool sendEmergencyStop();
    bool sendEmergencyRelease();
    bool sendSetStatusMode(uint8_t mode);
    
private:
    void receiveLoop();
    bool findFrameHeader();
    void processStatusPacket(const StatusPacket& packet);
    void parseCoordinateAndAxis0(const StatusPacket& packet, RobotStatus& status);
    void parseAxisAngles2(const StatusPacket& packet, RobotStatus& status);
    void parseAxisAngles3AndWorkspace(const StatusPacket& packet, RobotStatus& status);
    void parsePWMAndAxis4(const StatusPacket& packet, RobotStatus& status);
    void parseFileInfo(const StatusPacket& packet, RobotStatus& status);
    
    LibSerial::SerialPort serial_port_;
    rclcpp::Logger logger_;
    StatusCallback status_callback_;
    std::thread receive_thread_;
    std::atomic<bool> receiving_;
    std::atomic<bool> connected_;
    
    RobotStatus current_status_;
    std::mutex status_mutex_;
};

} // namespace robot_driver

#endif // ROBOT_DRIVER_SERIAL_COMMUNICATION_HPP