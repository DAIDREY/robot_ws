// src/serial_communication.cpp
#include "serial_communication.hpp"
#include <thread>
#include <chrono>
#include <cstring>
#include <iostream>

namespace robot_driver
{

SerialCommunication::SerialCommunication(rclcpp::Logger logger)
    : logger_(logger), receiving_(false), connected_(false)
{
    // 正确初始化状态结构，避免memset警告
    current_status_ = RobotStatus{};
    RCLCPP_DEBUG(logger_, "SerialCommunication构造函数完成");
}

SerialCommunication::~SerialCommunication()
{
    disconnect();
    RCLCPP_DEBUG(logger_, "SerialCommunication析构函数完成");
}

bool SerialCommunication::connect(const std::string& port, unsigned int baudrate)
{
    try {
        if (connected_) {
            RCLCPP_WARN(logger_, "串口已连接，先断开现有连接");
            disconnect();
        }

#ifdef USE_MOCK_SERIAL
        // 模拟串口连接
        RCLCPP_INFO(logger_, "使用模拟串口连接: %s, 波特率: %d", port.c_str(), baudrate);
        connected_ = true;
        return true;
#else
        // 实际串口连接 (需要libserial)
        serial_port_.Open(port);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        // 修复：使用正确的libserial API
        serial_port_.SetVTime(10); // 设置VTIME (1秒)
        serial_port_.SetVMin(0);   // 设置VMIN
        
        connected_ = true;
        RCLCPP_INFO(logger_, "真实串口连接成功: %s, 波特率: %d", port.c_str(), baudrate);
        return true;
#endif
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "串口连接异常: %s", e.what());
        connected_ = false;
        return false;
    }
}

void SerialCommunication::disconnect()
{
    if (receiving_) {
        stopStatusReceiving();
    }
    
#ifdef USE_MOCK_SERIAL
    // 模拟断开连接
    if (connected_) {
        RCLCPP_INFO(logger_, "模拟串口连接已断开");
    }
#else
    if (serial_port_.IsOpen()) {
        serial_port_.Close();
        RCLCPP_INFO(logger_, "真实串口连接已断开");
    }
#endif
    
    connected_ = false;
}

bool SerialCommunication::isConnected() const
{
#ifdef USE_MOCK_SERIAL
    return connected_;
#else
    return connected_ && serial_port_.IsOpen();
#endif
}

bool SerialCommunication::sendCommand(const std::vector<uint8_t>& command)
{
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "串口未连接，无法发送指令");
        return false;
    }
    
    if (command.size() != 48) {
        RCLCPP_ERROR(logger_, "指令长度错误，期望48字节，实际%zu字节", command.size());
        return false;
    }
    
    try {
#ifdef USE_MOCK_SERIAL
        // 模拟发送指令
        RCLCPP_DEBUG(logger_, "模拟发送指令成功，长度: %zu 字节", command.size());
        
        // 打印指令内容（调试用）
        if (RCLCPP_DEBUG_ENABLED(logger_)) {
            std::string hex_str = "";
            for (size_t i = 0; i < std::min(command.size(), size_t(16)); ++i) {
                char hex_char[4];
                snprintf(hex_char, sizeof(hex_char), "%02X ", command[i]);
                hex_str += hex_char;
            }
            if (command.size() > 16) hex_str += "...";
            RCLCPP_DEBUG(logger_, "指令内容: %s", hex_str.c_str());
        }
        
        // 验证指令格式
        if ((command[0] == 0xee && command[47] == 0xef) ||  // 运动指令
            (command[0] == 0xfc && command[47] == 0xfd)) {  // 控制指令
            RCLCPP_DEBUG(logger_, "指令格式验证通过");
        } else {
            RCLCPP_WARN(logger_, "指令格式可能错误: 帧头=0x%02X, 帧尾=0x%02X", 
                       command[0], command[47]);
        }
        
        return true;
#else
        // 修复：使用正确的libserial Write API
        std::string data_str(command.begin(), command.end());
        serial_port_.Write(data_str);
        serial_port_.DrainWriteBuffer();
        
        RCLCPP_DEBUG(logger_, "发送指令成功，长度: %zu 字节", command.size());
        return true;
#endif
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "发送指令失败: %s", e.what());
        return false;
    }
}

bool SerialCommunication::sendBytes(const uint8_t* data, size_t length)
{
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "串口未连接，无法发送数据");
        return false;
    }
    
    try {
#ifdef USE_MOCK_SERIAL
        // 模拟发送数据
        RCLCPP_DEBUG(logger_, "模拟发送数据成功，长度: %zu 字节", length);
        return true;
#else
        // 修复：转换为string后发送
        std::string data_str(reinterpret_cast<const char*>(data), length);
        serial_port_.Write(data_str);
        serial_port_.DrainWriteBuffer();
        
        RCLCPP_DEBUG(logger_, "发送数据成功，长度: %zu 字节", length);
        return true;
#endif
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "发送数据失败: %s", e.what());
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
#ifdef USE_MOCK_SERIAL
    // 模拟接收状态数据
    int packet_counter = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(logger_, "开始模拟状态接收循环 (16ms周期)");
    
    while (receiving_ && isConnected()) {
        try {
            // 模拟生成状态数据
            StatusPacket packet;
            packet.header = 0xce;
            packet.flag = packet_counter % 5; // 轮询不同类型的状态包 (0-4)
            packet.tail = 0xcf;
            
            // 生成模拟数据
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
            
            switch (packet.flag) {
                case 0: { // 坐标和关节角度
                    // 模拟缓慢变化的坐标数据
                    double t = elapsed / 1000.0; // 秒
                    uint16_t x = static_cast<uint16_t>((200.0 + 50.0 * sin(t * 0.1)) * 10); // X坐标
                    uint16_t y = static_cast<uint16_t>((150.0 + 30.0 * cos(t * 0.15)) * 10); // Y坐标
                    uint16_t z = static_cast<uint16_t>((300.0 + 20.0 * sin(t * 0.2)) * 10); // Z坐标
                    uint16_t a0 = static_cast<uint16_t>((10.0 * sin(t * 0.05)) * 100); // 关节0角度
                    
                    *(uint16_t*)&packet.data[0] = x;
                    *(uint16_t*)&packet.data[2] = y;
                    *(uint16_t*)&packet.data[4] = z;
                    *(uint16_t*)&packet.data[6] = a0;
                    break;
                }
                case 1: { // 关节角度2
                    double t = elapsed / 1000.0;
                    uint16_t a1 = static_cast<uint16_t>((20.0 * cos(t * 0.08)) * 100); // 关节1
                    uint16_t a2 = static_cast<uint16_t>((15.0 * sin(t * 0.12)) * 100); // 关节2
                    uint16_t b0 = static_cast<uint16_t>((5.0 * sin(t * 0.06)) * 100);  // B0姿态
                    uint16_t b1 = static_cast<uint16_t>((8.0 * cos(t * 0.09)) * 100);  // B1姿态
                    
                    *(uint16_t*)&packet.data[0] = a1;
                    *(uint16_t*)&packet.data[2] = a2;
                    *(uint16_t*)&packet.data[4] = b0;
                    *(uint16_t*)&packet.data[6] = b1;
                    break;
                }
                case 2: { // 关节角度3和工作台坐标
                    double t = elapsed / 1000.0;
                    uint16_t aw = static_cast<uint16_t>((25.0 * sin(t * 0.07)) * 100); // AW轴
                    uint16_t wk_x = static_cast<uint16_t>(100.0 * 10); // 工作台X
                    uint16_t wk_y = static_cast<uint16_t>(200.0 * 10); // 工作台Y
                    uint16_t wk_z = static_cast<uint16_t>(50.0 * 10);  // 工作台Z
                    
                    *(uint16_t*)&packet.data[0] = aw;
                    *(uint16_t*)&packet.data[2] = wk_x;
                    *(uint16_t*)&packet.data[4] = wk_y;
                    *(uint16_t*)&packet.data[6] = wk_z;
                    break;
                }
                case 3: { // PWM和关节角度4
                    double t = elapsed / 1000.0;
                    uint16_t pwm = static_cast<uint16_t>(1500 + 200 * sin(t * 0.1)); // PWM值
                    uint16_t w0 = static_cast<uint16_t>((12.0 * cos(t * 0.11)) * 100); // W0轴
                    uint16_t w1 = static_cast<uint16_t>((18.0 * sin(t * 0.13)) * 100); // W1轴
                    
                    *(uint16_t*)&packet.data[2] = pwm;
                    *(uint16_t*)&packet.data[4] = w0;
                    *(uint16_t*)&packet.data[6] = w1;
                    break;
                }
                case 4: { // 文件运行信息
                    uint32_t file_pos = static_cast<uint32_t>(elapsed / 10); // 模拟文件位置
                    uint16_t buffer_idx = static_cast<uint16_t>(packet_counter % 9600); // 缓冲区索引
                    uint8_t structure = 9; // 默认结构模式
                    uint8_t error = 0;     // 无错误
                    
                    *(uint32_t*)&packet.data[0] = file_pos;
                    *(uint16_t*)&packet.data[4] = buffer_idx;
                    packet.data[6] = structure;
                    packet.data[7] = error;
                    break;
                }
                default:
                    memset(packet.data, 0, sizeof(packet.data));
                    break;
            }
            
            processStatusPacket(packet);
            packet_counter++;
            
            // 每1000个包打印一次统计信息
            if (packet_counter % 1000 == 0) {
                RCLCPP_DEBUG(logger_, "已处理 %d 个状态包，运行时间: %.1f秒", 
                           packet_counter, elapsed / 1000.0);
            }
            
            // 模拟16ms接收周期
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "模拟状态接收异常: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    RCLCPP_INFO(logger_, "模拟状态接收循环结束，总计处理 %d 个包", packet_counter);
#else
    // 实际串口接收
    StatusPacket packet;
    int packet_counter = 0;
    
    RCLCPP_INFO(logger_, "开始真实状态接收循环");
    
    while (receiving_ && isConnected()) {
        try {
            // 查找帧头
            if (findFrameHeader()) {
                // 修复：使用正确的libserial Read API
                if (static_cast<size_t>(serial_port_.GetNumberOfBytesAvailable()) >= sizeof(StatusPacket) - 1) {
                    std::string data_str;
                    serial_port_.Read(data_str, sizeof(packet) - 1, 100); // 100ms超时
                    
                    if (data_str.size() == sizeof(packet) - 1) {
                        packet.header = 0xce;
                        memcpy(&packet.data, data_str.data(), sizeof(packet) - 1);
                        
                        // 验证帧尾
                        if (packet.tail == 0xcf) {
                            processStatusPacket(packet);
                            packet_counter++;
                            
                            if (packet_counter % 500 == 0) {
                                RCLCPP_DEBUG(logger_, "已处理 %d 个真实状态包", packet_counter);
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
    
    RCLCPP_INFO(logger_, "真实状态接收循环结束，总计处理 %d 个包", packet_counter);
#endif
}

bool SerialCommunication::findFrameHeader()
{
#ifdef USE_MOCK_SERIAL
    return true; // 模拟模式始终返回true
#else
    std::string byte_str;
    while (receiving_ && serial_port_.GetNumberOfBytesAvailable() > 0) {
        try {
            serial_port_.Read(byte_str, 1, 100); // 100ms超时
            if (!byte_str.empty() && static_cast<uint8_t>(byte_str[0]) == 0xce) {
                return true;
            }
        } catch (const std::exception& e) {
            return false;
        }
    }
    return false;
#endif
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
        case 4:  // 文件运行信息1
        case 5:  // 文件运行信息2
        case 6:  // 文件运行信息3
        case 7:  // 文件运行信息4
        case 8:  // 触发运行循环设置信息
            parseFileInfo(packet, current_status_);
            break;
        default:
            RCLCPP_DEBUG(logger_, "未知状态包类型: %d", packet.flag);
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
    
    RCLCPP_DEBUG(logger_, "坐标: (%.1f, %.1f, %.1f)mm, 轴0: %.2f°", 
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
    
    RCLCPP_DEBUG(logger_, "轴1: %.2f°, 轴2: %.2f°, B0: %.2f°, B1: %.2f°", 
                status.joint_angles[1], status.joint_angles[2], 
                status.orientation[0], status.orientation[1]);
}

void SerialCommunication::parseAxisAngles3AndWorkspace(const StatusPacket& packet, RobotStatus& status)
{
    // 解析AW轴角度
    uint16_t aw_raw = static_cast<uint16_t>(packet.data[0]) | (static_cast<uint16_t>(packet.data[1]) << 8);
    status.joint_angles[5] = static_cast<double>(aw_raw) / 100.0;  // AW轴
    
    // 解析工作台坐标
    uint16_t wk_x0 = static_cast<uint16_t>(packet.data[2]) | (static_cast<uint16_t>(packet.data[3]) << 8);
    uint16_t wk_y0 = static_cast<uint16_t>(packet.data[4]) | (static_cast<uint16_t>(packet.data[5]) << 8);
    uint16_t wk_z0 = static_cast<uint16_t>(packet.data[6]) | (static_cast<uint16_t>(packet.data[7]) << 8);
    
    status.workspace_origin[0] = static_cast<double>(wk_x0) / 10.0;
    status.workspace_origin[1] = static_cast<double>(wk_y0) / 10.0;
    status.workspace_origin[2] = static_cast<double>(wk_z0) / 10.0;
    
    RCLCPP_DEBUG(logger_, "AW轴: %.2f°, 工作台原点: (%.1f, %.1f, %.1f)mm", 
                status.joint_angles[5], status.workspace_origin[0], 
                status.workspace_origin[1], status.workspace_origin[2]);
}

void SerialCommunication::parsePWMAndAxis4(const StatusPacket& packet, RobotStatus& status)
{
    // 解析PWM值
    status.pwm_value = static_cast<uint16_t>(packet.data[2]) | (static_cast<uint16_t>(packet.data[3]) << 8);
    
    // 解析轴角度 W0, W1
    uint16_t w0_raw = static_cast<uint16_t>(packet.data[4]) | (static_cast<uint16_t>(packet.data[5]) << 8);
    uint16_t w1_raw = static_cast<uint16_t>(packet.data[6]) | (static_cast<uint16_t>(packet.data[7]) << 8);
    
    status.joint_angles[3] = static_cast<double>(w0_raw) / 100.0;  // W0
    status.joint_angles[4] = static_cast<double>(w1_raw) / 100.0;  // W1
    status.orientation[2] = status.joint_angles[4];  // W轴对应orientation的W
    
    RCLCPP_DEBUG(logger_, "PWM: %d, W0: %.2f°, W1: %.2f°", 
                status.pwm_value, status.joint_angles[3], status.joint_angles[4]);
}

void SerialCommunication::parseFileInfo(const StatusPacket& packet, RobotStatus& status)
{
    switch (packet.flag) {
        case 4:  // 文件运行信息1
            // 解析文件运行位置 (32位整数)
            status.file_position = static_cast<uint32_t>(packet.data[0]) |
                                 (static_cast<uint32_t>(packet.data[1]) << 8) |
                                 (static_cast<uint32_t>(packet.data[2]) << 16) |
                                 (static_cast<uint32_t>(packet.data[3]) << 24);
            
            // 解析缓冲区索引
            status.buffer_index = static_cast<uint16_t>(packet.data[4]) | 
                                (static_cast<uint16_t>(packet.data[5]) << 8);
            
            // 解析结构和错误代码
            status.structure_mode = packet.data[6];
            status.error_code = packet.data[7];
            
            RCLCPP_DEBUG(logger_, "文件位置: %u, 缓冲区: %d, 结构: %d, 错误: %d", 
                        status.file_position, status.buffer_index, 
                        status.structure_mode, status.error_code);
            
            // 错误检查
            if (status.error_code != 0) {
                RCLCPP_WARN(logger_, "检测到错误代码: %d", status.error_code);
            }
            
            // 缓冲区状态检查
            if (status.buffer_index > 9000) {
                RCLCPP_WARN(logger_, "缓冲区接近满载: %d/9600", status.buffer_index);
            }
            break;
            
        default:
            RCLCPP_DEBUG(logger_, "其他文件信息类型 %d", packet.flag);
            break;
    }
}

// 控制指令实现
bool SerialCommunication::sendRobotReset()
{
    std::vector<uint8_t> command(48, 0);
    command[0] = 0xfc;   // 控制指令帧头
    command[1] = 12;     // 指令a
    command[2] = 3;      // 指令b
    command[47] = 0xfd;  // 控制指令帧尾
    
    RCLCPP_INFO(logger_, "发送机器人复位指令");
    return sendCommand(command);
}

bool SerialCommunication::sendConnectorReset()
{
    std::vector<uint8_t> command(48, 0);
    command[0] = 0xfc;   // 控制指令帧头
    command[1] = 12;     // 指令a
    command[2] = 6;      // 指令b
    command[47] = 0xfd;  // 控制指令帧尾
    
    RCLCPP_INFO(logger_, "发送连接器复位指令");
    return sendCommand(command);
}

bool SerialCommunication::sendEmergencyStop()
{
    std::vector<uint8_t> command(48, 0);
    command[0] = 0xfc;   // 控制指令帧头
    command[1] = 12;     // 指令a
    command[2] = 8;      // 指令b (急停)
    command[3] = 0;      // 固定值
    command[47] = 0xfd;  // 控制指令帧尾
    
    RCLCPP_WARN(logger_, "发送急停指令");
    return sendCommand(command);
}

bool SerialCommunication::sendEmergencyRelease()
{
    std::vector<uint8_t> command(48, 0);
    command[0] = 0xfc;   // 控制指令帧头
    command[1] = 12;     // 指令a
    command[2] = 9;      // 指令b (解除急停)
    command[3] = 0;      // 固定值
    command[47] = 0xfd;  // 控制指令帧尾
    
    RCLCPP_INFO(logger_, "发送解除急停指令");
    return sendCommand(command);
}

bool SerialCommunication::sendSetStatusMode(uint8_t mode)
{
    std::vector<uint8_t> command(48, 0);
    command[0] = 0xfc;   // 控制指令帧头
    command[1] = 30;     // 指令a
    command[2] = 10;     // 指令b (状态信息控制)
    command[3] = mode;   // 0: 仅文件信息, 1: +XYZ, 2: 全部
    command[47] = 0xfd;  // 控制指令帧尾
    
    const char* mode_desc[] = {"仅文件信息", "文件信息+XYZ", "全部状态信息"};
    RCLCPP_INFO(logger_, "设置状态信息模式: %d (%s)", 
               mode, (mode < 3) ? mode_desc[mode] : "未知模式");
    
    return sendCommand(command);
}

} // namespace robot_driver