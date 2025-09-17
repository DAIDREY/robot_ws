// src/robot_driver/src/serial_communication.cpp
#include "serial_communication.hpp"
#include <cstring>
#include <chrono>
#include <thread>

namespace robot_driver
{

SerialCommunication::SerialCommunication(rclcpp::Logger logger)
    : logger_(logger), receiving_(false), connected_(false)
{
    RCLCPP_INFO(logger_, "初始化串口通信");
}

SerialCommunication::~SerialCommunication()
{
    disconnect();
}

bool SerialCommunication::connect(const std::string& port, unsigned int baudrate)
{
    try {
        serial_port_.Open(port);
        
        LibSerial::BaudRate libserial_baudrate;
        switch(baudrate) {
            case 1200: libserial_baudrate = LibSerial::BaudRate::BAUD_1200; break;
            case 2400: libserial_baudrate = LibSerial::BaudRate::BAUD_2400; break;
            case 4800: libserial_baudrate = LibSerial::BaudRate::BAUD_4800; break;
            case 9600: libserial_baudrate = LibSerial::BaudRate::BAUD_9600; break;
            case 19200: libserial_baudrate = LibSerial::BaudRate::BAUD_19200; break;
            case 38400: libserial_baudrate = LibSerial::BaudRate::BAUD_38400; break;
            case 57600: libserial_baudrate = LibSerial::BaudRate::BAUD_57600; break;
            case 115200: libserial_baudrate = LibSerial::BaudRate::BAUD_115200; break;
            case 230400: libserial_baudrate = LibSerial::BaudRate::BAUD_230400; break;
            default:
                RCLCPP_ERROR(logger_, "不支持的波特率: %d", baudrate);
                return false;
        }
        
        serial_port_.SetBaudRate(libserial_baudrate);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        
        connected_ = true;
        RCLCPP_INFO(logger_, "串口连接成功: %s, 波特率: %d", port.c_str(), baudrate);
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "串口连接失败: %s, 端口: %s, 波特率: %d", e.what(), port.c_str(), baudrate);
        connected_ = false;
        return false;
    }
}

void SerialCommunication::disconnect()
{
    stopStatusReceiving();
    
    if (connected_) {
        try {
            serial_port_.Close();
            connected_ = false;
            RCLCPP_INFO(logger_, "串口连接已断开");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "串口断开异常: %s", e.what());
        }
    }
}

bool SerialCommunication::isConnected() const
{
    return connected_;
}

// IEEE 754浮点数转换为字节数组 (小端序)
std::vector<uint8_t> SerialCommunication::floatToBytes(float value)
{
    std::vector<uint8_t> bytes(4);
    std::memcpy(bytes.data(), &value, 4);
    return bytes;
}

bool SerialCommunication::sendLinearMove(const Position3D& position, const Orientation& orientation, 
                                       double speed, uint16_t pwm)
{
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "串口未连接，无法发送直线插补指令");
        return false;
    }

    // 创建48字节直线插补指令
    std::vector<uint8_t> command(48, 0);
    
    // 设置帧头和指令代码
    command[0] = 0xEE;  // 运动指令帧头
    command[1] = 0x31;  // 字符'1' (直线插补)
    command[2] = 0x01;  // 指令b (标准直线插补)
    
    // 设置目标坐标 (浮点数1、2、3)
    auto xtBytes = floatToBytes(static_cast<float>(position.x));
    auto ytBytes = floatToBytes(static_cast<float>(position.y));
    auto ztBytes = floatToBytes(static_cast<float>(position.z));
    
    std::copy(xtBytes.begin(), xtBytes.end(), command.begin() + 3);   // X坐标
    std::copy(ytBytes.begin(), ytBytes.end(), command.begin() + 7);   // Y坐标
    std::copy(ztBytes.begin(), ztBytes.end(), command.begin() + 11);  // Z坐标
    
    // 设置姿态角度 (浮点数4、5、6)
    auto b0Bytes = floatToBytes(static_cast<float>(orientation.b0));
    auto b1Bytes = floatToBytes(static_cast<float>(orientation.b1));
    auto wBytes = floatToBytes(static_cast<float>(orientation.w));
    
    std::copy(b0Bytes.begin(), b0Bytes.end(), command.begin() + 15);  // B0姿态
    std::copy(b1Bytes.begin(), b1Bytes.end(), command.begin() + 19);  // B1姿态
    std::copy(wBytes.begin(), wBytes.end(), command.begin() + 23);    // W姿态
    
    // 设置PWM值 (浮点数7)
    auto pwmBytes = floatToBytes(static_cast<float>(pwm));
    std::copy(pwmBytes.begin(), pwmBytes.end(), command.begin() + 27);
    
    // 外部轴角度值设为0 (浮点数8、9)
    auto zeroBytes = floatToBytes(0.0f);
    std::copy(zeroBytes.begin(), zeroBytes.end(), command.begin() + 31);  // E0
    std::copy(zeroBytes.begin(), zeroBytes.end(), command.begin() + 35);  // E1
    std::copy(zeroBytes.begin(), zeroBytes.end(), command.begin() + 39);  // 浮点数10
    
    // 设置运动速度 (浮点数11, mm/分钟)
    auto speedBytes = floatToBytes(static_cast<float>(speed));
    std::copy(speedBytes.begin(), speedBytes.end(), command.begin() + 43);
    
    // 设置帧尾
    command[47] = 0xEF;
    
    RCLCPP_INFO(logger_, "发送直线插补指令: X=%.1f, Y=%.1f, Z=%.1f, 速度=%.1fmm/分", 
               position.x, position.y, position.z, speed);
    
    return sendCommand(command);
}

bool SerialCommunication::sendJointMove(const JointAngles& joint_angles, double speed, 
                                       uint8_t buffer_control, uint16_t pwm)
{
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "串口未连接，无法发送轴角度插补指令");
        return false;
    }

    // 创建48字节轴角度插补指令
    std::vector<uint8_t> command(48, 0);
    
    // 设置帧头和指令代码
    command[0] = 0xEE;  // 运动指令帧头
    command[1] = 0x33;  // 字符'3' (轴角度插补)
    command[2] = buffer_control;  // 启停缓冲控制
    
    // 设置6个关节角度 (浮点数1-6)
    auto a0Bytes = floatToBytes(static_cast<float>(joint_angles.angles[0]));  // 关节1
    auto a1Bytes = floatToBytes(static_cast<float>(joint_angles.angles[1]));  // 关节2
    auto a2Bytes = floatToBytes(static_cast<float>(joint_angles.angles[2]));  // 关节3
    auto w0Bytes = floatToBytes(static_cast<float>(joint_angles.angles[3]));  // 关节4
    auto w1Bytes = floatToBytes(static_cast<float>(joint_angles.angles[4]));  // 关节5
    auto awBytes = floatToBytes(static_cast<float>(joint_angles.angles[5]));  // 关节6
    
    std::copy(a0Bytes.begin(), a0Bytes.end(), command.begin() + 3);   // a0
    std::copy(a1Bytes.begin(), a1Bytes.end(), command.begin() + 7);   // a1
    std::copy(a2Bytes.begin(), a2Bytes.end(), command.begin() + 11);  // a2
    std::copy(w0Bytes.begin(), w0Bytes.end(), command.begin() + 15);  // w0
    std::copy(w1Bytes.begin(), w1Bytes.end(), command.begin() + 19);  // w1
    std::copy(awBytes.begin(), awBytes.end(), command.begin() + 23);  // aw
    
    // 设置PWM值 (浮点数7)
    auto pwmBytes = floatToBytes(static_cast<float>(pwm));
    std::copy(pwmBytes.begin(), pwmBytes.end(), command.begin() + 27);
    
    // 外部轴角度值设为0 (浮点数8、9)
    auto zeroBytes = floatToBytes(0.0f);
    std::copy(zeroBytes.begin(), zeroBytes.end(), command.begin() + 31);  // E0
    std::copy(zeroBytes.begin(), zeroBytes.end(), command.begin() + 35);  // E1
    std::copy(zeroBytes.begin(), zeroBytes.end(), command.begin() + 39);  // 浮点数10
    
    // 设置转速 (浮点数11, 度/秒)
    auto speedBytes = floatToBytes(static_cast<float>(speed));
    std::copy(speedBytes.begin(), speedBytes.end(), command.begin() + 43);
    
    // 设置帧尾
    command[47] = 0xEF;
    
    RCLCPP_INFO(logger_, "发送轴角度插补指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]°, 速度=%.1f°/s", 
               joint_angles.angles[0], joint_angles.angles[1], joint_angles.angles[2],
               joint_angles.angles[3], joint_angles.angles[4], joint_angles.angles[5], speed);
    
    return sendCommand(command);
}

bool SerialCommunication::sendCommand(const std::vector<uint8_t>& command)
{
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "串口未连接，无法发送指令");
        return false;
    }
    
    if (command.size() != 48) {
        RCLCPP_ERROR(logger_, "指令长度错误: %zu，期望48字节", command.size());
        return false;
    }
    
    try {
        std::string data_str(command.begin(), command.end());
        serial_port_.Write(data_str);
        serial_port_.DrainWriteBuffer();
        
        RCLCPP_DEBUG(logger_, "发送指令成功，长度: %zu 字节", command.size());
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "发送指令失败: %s", e.what());
        return false;
    }
}

void SerialCommunication::setStatusCallback(StatusCallback callback)
{
    status_callback_ = callback;
    RCLCPP_DEBUG(logger_, "状态回调函数已设置");
}

void SerialCommunication::startStatusReceiving()
{
    if (receiving_ || !isConnected()) {
        RCLCPP_WARN(logger_, "状态接收已经启动或串口未连接");
        return;
    }
    
    receiving_ = true;
    receive_thread_ = std::thread(&SerialCommunication::receiveLoop, this);
    RCLCPP_INFO(logger_, "状态接收线程已启动");
}

void SerialCommunication::stopStatusReceiving()
{
    if (!receiving_) {
        return;
    }
    
    receiving_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    RCLCPP_INFO(logger_, "状态接收线程已停止");
}

void SerialCommunication::receiveLoop()
{
    StatusPacket packet;
    int packet_counter = 0;
    
    RCLCPP_INFO(logger_, "开始状态接收循环");
    
    while (receiving_ && isConnected()) {
        try {
            // 查找帧头
            if (findFrameHeader()) {
                // 读取状态包数据
                if (static_cast<size_t>(serial_port_.GetNumberOfBytesAvailable()) >= sizeof(StatusPacket) - 1) {
                    std::string data_str;
                    serial_port_.Read(data_str, sizeof(packet) - 1, 100); // 100ms超时
                    
                    if (data_str.size() == sizeof(packet) - 1) {
                        packet.header = 0xCE;
                        std::memcpy(&packet.data, data_str.data(), sizeof(packet) - 1);
                        
                        // 验证帧尾
                        if (packet.tail == 0xCF) {
                            processStatusPacket(packet);
                            packet_counter++;
                            
                            if (packet_counter % 500 == 0) {
                                RCLCPP_DEBUG(logger_, "已处理 %d 个状态包", packet_counter);
                            }
                        } else {
                            RCLCPP_DEBUG(logger_, "无效的状态包帧尾: 0x%02x", packet.tail);
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "状态接收异常: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    RCLCPP_INFO(logger_, "状态接收循环结束，总计处理 %d 个包", packet_counter);
}

bool SerialCommunication::findFrameHeader()
{
    std::string byte_str;
    while (receiving_ && serial_port_.GetNumberOfBytesAvailable() > 0) {
        try {
            serial_port_.Read(byte_str, 1, 100); // 100ms超时
            if (!byte_str.empty() && static_cast<uint8_t>(byte_str[0]) == 0xCE) {
                return true;
            }
        } catch (const std::exception& e) {
            return false;
        }
    }
    return false;
}

void SerialCommunication::processStatusPacket(const StatusPacket& packet)
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 更新时间戳
    current_status_.timestamp = std::chrono::system_clock::now();
    
    switch (packet.flag) {
        case 0:  // 坐标值、轴角度1
            parseCoordinateAndAxis0(packet, current_status_);
            break;
        case 1:  // 轴角度值2
            parseAxisAngles2(packet, current_status_);
            break;
        case 2:  // 轴角度值3、工作台坐标
            parseAxisAngles3AndWorkspace(packet, current_status_);
            break;
        case 3:  // PWM及轴角度4
            parsePWMAndAxis4(packet, current_status_);
            break;
        default:
            RCLCPP_DEBUG(logger_, "忽略状态包类型: %d", packet.flag);
            break;
    }
    
    // 调用回调函数
    if (status_callback_) {
        status_callback_(current_status_);
    }
}

void SerialCommunication::parseCoordinateAndAxis0(const StatusPacket& packet, RobotStatus& status)
{
    // 解析X, Y, Z坐标 (小端序，除以10得到实际值mm)
    uint16_t x_raw = static_cast<uint16_t>(packet.data[0]) | (static_cast<uint16_t>(packet.data[1]) << 8);
    uint16_t y_raw = static_cast<uint16_t>(packet.data[2]) | (static_cast<uint16_t>(packet.data[3]) << 8);
    uint16_t z_raw = static_cast<uint16_t>(packet.data[4]) | (static_cast<uint16_t>(packet.data[5]) << 8);
    
    status.x = static_cast<double>(x_raw) / 10.0;
    status.y = static_cast<double>(y_raw) / 10.0;
    status.z = static_cast<double>(z_raw) / 10.0;
    
    // 解析轴0角度 (除以100得到实际值度)
    uint16_t a0_raw = static_cast<uint16_t>(packet.data[6]) | (static_cast<uint16_t>(packet.data[7]) << 8);
    status.joint_angles[0] = static_cast<double>(a0_raw) / 100.0;
    
    RCLCPP_INFO(logger_, "坐标: (%.1f, %.1f, %.1f)mm, 轴0: %.2f°", 
                status.x, status.y, status.z, status.joint_angles[0]);
}

void SerialCommunication::parseAxisAngles2(const StatusPacket& packet, RobotStatus& status)
{
    // 解析轴1, 轴2角度
    uint16_t a1_raw = static_cast<uint16_t>(packet.data[0]) | (static_cast<uint16_t>(packet.data[1]) << 8);
    uint16_t a2_raw = static_cast<uint16_t>(packet.data[2]) | (static_cast<uint16_t>(packet.data[3]) << 8);
    
    status.joint_angles[1] = static_cast<double>(a1_raw) / 100.0;
    status.joint_angles[2] = static_cast<double>(a2_raw) / 100.0;
    
    // 解析B0, B1姿态角
    uint16_t b0_raw = static_cast<uint16_t>(packet.data[4]) | (static_cast<uint16_t>(packet.data[5]) << 8);
    uint16_t b1_raw = static_cast<uint16_t>(packet.data[6]) | (static_cast<uint16_t>(packet.data[7]) << 8);
    
    status.orientation[0] = static_cast<double>(b0_raw) / 100.0;  // B0
    status.orientation[1] = static_cast<double>(b1_raw) / 100.0;  // B1
    
    RCLCPP_INFO(logger_, "轴1: %.2f°, 轴2: %.2f°, B0: %.2f°, B1: %.2f°", 
                status.joint_angles[1], status.joint_angles[2], 
                status.orientation[0], status.orientation[1]);
}

void SerialCommunication::parseAxisAngles3AndWorkspace(const StatusPacket& packet, RobotStatus& status)
{
    // 解析AW轴角度 (关节5)
    uint16_t aw_raw = static_cast<uint16_t>(packet.data[0]) | (static_cast<uint16_t>(packet.data[1]) << 8);
    status.joint_angles[5] = static_cast<double>(aw_raw) / 100.0;  // AW轴 (关节5)
    
    // 解析工作台坐标
    uint16_t wk_x = static_cast<uint16_t>(packet.data[2]) | (static_cast<uint16_t>(packet.data[3]) << 8);
    uint16_t wk_y = static_cast<uint16_t>(packet.data[4]) | (static_cast<uint16_t>(packet.data[5]) << 8);
    uint16_t wk_z = static_cast<uint16_t>(packet.data[6]) | (static_cast<uint16_t>(packet.data[7]) << 8);
    
    status.workspace_origin[0] = static_cast<double>(wk_x) / 10.0;
    status.workspace_origin[1] = static_cast<double>(wk_y) / 10.0;
    status.workspace_origin[2] = static_cast<double>(wk_z) / 10.0;
    
    RCLCPP_INFO(logger_, "关节5(AW): %.2f°, 工作台原点: (%.1f, %.1f, %.1f)mm", 
                status.joint_angles[5], status.workspace_origin[0], 
                status.workspace_origin[1], status.workspace_origin[2]);
}

void SerialCommunication::parsePWMAndAxis4(const StatusPacket& packet, RobotStatus& status)
{
    // 解析PWM值
    status.pwm_value = static_cast<uint16_t>(packet.data[2]) | (static_cast<uint16_t>(packet.data[3]) << 8);
    
    // 解析轴角度 W0, W1 (关节3, 关节4)
    uint16_t w0_raw = static_cast<uint16_t>(packet.data[4]) | (static_cast<uint16_t>(packet.data[5]) << 8);
    uint16_t w1_raw = static_cast<uint16_t>(packet.data[6]) | (static_cast<uint16_t>(packet.data[7]) << 8);
    
    status.joint_angles[3] = static_cast<double>(w0_raw) / 100.0;  // W0 (关节3)
    status.joint_angles[4] = static_cast<double>(w1_raw) / 100.0;  // W1 (关节4)
    status.orientation[2] = status.joint_angles[4];  // W轴对应orientation的W
    
    RCLCPP_INFO(logger_, "PWM: %d, 关节3(W0): %.2f°, 关节4(W1): %.2f°", 
                status.pwm_value, status.joint_angles[3], status.joint_angles[4]);
}

RobotStatus SerialCommunication::getCurrentStatus() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

} // namespace robot_driver