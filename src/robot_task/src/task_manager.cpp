#include "task_manager.hpp"

namespace robot_task
{

TaskManager::TaskManager() : Node("task_manager")
{
    // 声明参数
    this->declare_parameter("pre_grasp_height", 0.10);
    this->declare_parameter("grasp_height_offset", 0.02);
    this->declare_parameter("gripper_length", 0.08);        // 保留夹爪长度偏移
    this->declare_parameter("lift_height", 0.10);
    this->declare_parameter("place_offset_x", 0.2);
    this->declare_parameter("place_offset_y", -0.2);
    this->declare_parameter("place_offset_z", 0.1);
    this->declare_parameter("wait_time_at_object", 3.0);    // 在物体位置等待3秒
    
    // 新增：初始位置参数
    this->declare_parameter("home_position_x", 0.344);        // 初始位置：机械臂前方30cm
    this->declare_parameter("home_position_y", 0.0);        // 初始位置：中央
    this->declare_parameter("home_position_z", 0.417);        // 初始位置：高度50cm
    this->declare_parameter("home_orientation_roll", 0.0);  // 初始姿态
    this->declare_parameter("home_orientation_pitch", 0.0);
    this->declare_parameter("home_orientation_yaw", 0.0);
    
    // 获取参数
    pre_grasp_height_ = this->get_parameter("pre_grasp_height").as_double();
    grasp_height_offset_ = this->get_parameter("grasp_height_offset").as_double();
    gripper_length_ = this->get_parameter("gripper_length").as_double();
    lift_height_ = this->get_parameter("lift_height").as_double();
    place_offset_x_ = this->get_parameter("place_offset_x").as_double();
    place_offset_y_ = this->get_parameter("place_offset_y").as_double();
    place_offset_z_ = this->get_parameter("place_offset_z").as_double();
    wait_time_at_object_ = this->get_parameter("wait_time_at_object").as_double();
    
    home_position_x_ = this->get_parameter("home_position_x").as_double();
    home_position_y_ = this->get_parameter("home_position_y").as_double();
    home_position_z_ = this->get_parameter("home_position_z").as_double();
    home_orientation_roll_ = this->get_parameter("home_orientation_roll").as_double();
    home_orientation_pitch_ = this->get_parameter("home_orientation_pitch").as_double();
    home_orientation_yaw_ = this->get_parameter("home_orientation_yaw").as_double();
    // 初始化状态
    task_active_ = false;
    object_detected_ = false;
    
    // 创建服务
    grasp_service_ = this->create_service<robot_task::srv::GraspObject>(
        "/grasp_object",
        std::bind(&TaskManager::graspObjectCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // 创建订阅者 - 订阅robot_visioner的ObjectPose
    object_pose_sub_ = this->create_subscription<robot_task::msg::ObjectPose>(
        "/robot_visioner/object_pose", 10,
        std::bind(&TaskManager::objectPoseCallback, this, std::placeholders::_1));
    
    // 创建发布者
    pose_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pose_target", 10);
    // gripper_command_pub_ = this->create_publisher<std_msgs::msg::String>(
    //     "/gripper_command", 10);
    status_pub_ = this->create_publisher<robot_task::msg::TaskStatus>(
        "/robot_task/status", 10);
    
    RCLCPP_INFO(this->get_logger(), "🤖 Task Manager 已启动，等待抓取请求...");
    RCLCPP_INFO(this->get_logger(), "📋 服务接口: /grasp_object");
    RCLCPP_INFO(this->get_logger(), "📍 监听物体姿态: /robot_visioner/object_pose");
    RCLCPP_INFO(this->get_logger(), "📤 发布控制指令: /pose_target, /gripper_command");
    
    // 发布初始状态
    publishStatus(0, "系统就绪，等待抓取请求");
}

geometry_msgs::msg::PoseStamped TaskManager::getHomePosition()
{
    geometry_msgs::msg::PoseStamped home_pose;
    home_pose.header.frame_id = "base_link";
    home_pose.header.stamp = this->now();
    
    // 设置初始位置
    home_pose.pose.position.x = home_position_x_;
    home_pose.pose.position.y = home_position_y_;
    home_pose.pose.position.z = home_position_z_;
    
    // 设置初始姿态
    tf2::Quaternion q;
    q.setRPY(home_orientation_roll_, home_orientation_pitch_, home_orientation_yaw_);
    home_pose.pose.orientation = tf2::toMsg(q);
    
    return home_pose;
}

bool TaskManager::moveToHomePosition()
{
    RCLCPP_INFO(this->get_logger(), "🏠 移动到初始位置...");
    
    auto home_pose = getHomePosition();
    pose_target_pub_->publish(home_pose);
    
    // 等待到达初始位置
    if (!waitForMotionComplete(8.0)) {
        RCLCPP_ERROR(this->get_logger(), "移动到初始位置失败");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ 已到达初始位置");
    return true;
}

void TaskManager::graspObjectCallback(
    const std::shared_ptr<robot_task::srv::GraspObject::Request> request,
    std::shared_ptr<robot_task::srv::GraspObject::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "🎯 收到抓取请求: %s", request->object_name.c_str());
    
    // 检查是否有任务正在进行
    if (task_active_) {
        response->success = false;
        response->message = "任务正在进行中，请稍后再试";
        RCLCPP_WARN(this->get_logger(), "⚠️ 任务正在进行中，拒绝新请求");
        return;
    }
    
    // 开始任务
    task_active_ = true;
    current_target_object_ = request->object_name;
    
    try {
        publishStatus(1, "正在搜索物体: " + request->object_name);
        
        // 等待物体出现
        robot_task::msg::ObjectPose object_pose;
        if (!waitForObject(request->object_name, object_pose)) {
            response->success = false;
            response->message = "未找到目标物体: " + request->object_name;
            publishStatus(4, "搜索失败 - 未找到物体");
            task_active_ = false;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ 找到物体: %s at (%.3f, %.3f, %.3f), 角度: %.1f°",
                   object_pose.object_name.c_str(),
                   object_pose.position.x, object_pose.position.y, object_pose.position.z,
                   object_pose.rotation_angle);
        
        publishStatus(2, "开始执行抓取");
        
        // 计算抓取姿态
        auto grasp_pose = calculateGraspPose(object_pose);
        
        // 执行抓取
        if (executeGraspSequence(grasp_pose)) {
            response->success = true;
            response->message = "抓取成功完成";
            response->object_pose = object_pose;
            publishStatus(3, "抓取完成");
            RCLCPP_INFO(this->get_logger(), "🎉 抓取任务成功完成!");
        } else {
            response->success = false;
            response->message = "抓取执行失败";
            publishStatus(4, "执行失败");
            RCLCPP_ERROR(this->get_logger(), "❌ 抓取执行失败");
        }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "抓取过程发生错误: " + std::string(e.what());
        publishStatus(4, "发生异常错误");
        RCLCPP_ERROR(this->get_logger(), "💥 抓取过程异常: %s", e.what());
    }
    
    // 重置状态
    task_active_ = false;
    current_target_object_.clear();
    object_detected_ = false;
}

void TaskManager::objectPoseCallback(const robot_task::msg::ObjectPose::SharedPtr msg)
{
    // 只处理当前目标物体，或者在没有任务时记录所有物体
    if (!task_active_ || msg->object_name == current_target_object_) {
        // 检查是否是新的物体或位置有显著变化
        static std::string last_object_name;
        static geometry_msgs::msg::Point last_position;
        static double last_rotation_angle;
        
        bool should_print = false;
        
        // 如果是不同的物体，或者位置/角度变化超过阈值，则打印
        if (msg->object_name != last_object_name ||
            std::abs(msg->position.x - last_position.x) > 0.02 ||
            std::abs(msg->position.y - last_position.y) > 0.02 ||
            std::abs(msg->position.z - last_position.z) > 0.02 ||
            std::abs(msg->rotation_angle - last_rotation_angle) > 5.0) {
            
            should_print = true;
            
            // 更新上次记录的值
            last_object_name = msg->object_name;
            last_position = msg->position;
            last_rotation_angle = msg->rotation_angle;
        }
        
        // 更新物体信息
        latest_object_pose_ = *msg;
        object_detected_ = true;
        
        // 只在需要时打印
        if (should_print) {
            RCLCPP_INFO(this->get_logger(), 
                "收到物体姿态: %s at (%.3f, %.3f, %.3f), 角度: %.1f°, 置信度: %.2f",
                msg->object_name.c_str(),
                msg->position.x, msg->position.y, msg->position.z,
                msg->rotation_angle, msg->confidence);
        }
            
        // 如果正在执行任务且是目标物体，记录日志
        if (task_active_ && msg->object_name == current_target_object_) {
            RCLCPP_INFO(this->get_logger(), "🎯 检测到目标物体: %s", msg->object_name.c_str());
        }
    }
}

bool TaskManager::waitForObject(const std::string& object_name, robot_task::msg::ObjectPose& pose)
{
    RCLCPP_INFO(this->get_logger(), "🔍 等待物体出现: %s", object_name.c_str());
    
    auto start_time = this->now();
    const auto timeout = rclcpp::Duration::from_seconds(15);
    
    // 注释掉重置检测标志
    // object_detected_ = false;
    
    while (rclcpp::ok() && (this->now() - start_time) < timeout) {
        // 简化条件检查 - 只检查物体名称匹配
        if (object_detected_ && latest_object_pose_.object_name == object_name) {
            pose = latest_object_pose_;
            RCLCPP_INFO(this->get_logger(), "✅ 找到物体: %s", object_name.c_str());
            return true;
        }
        
        // 处理挂起的ROS回调
        rclcpp::spin_some(this->get_node_base_interface());
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_WARN(this->get_logger(), "等待物体超时: %s", object_name.c_str());
    return false;
}

geometry_msgs::msg::PoseStamped TaskManager::calculateGraspPose(const robot_task::msg::ObjectPose& object_pose)
{
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = "base_link";
    grasp_pose.header.stamp = this->now();
    
    grasp_pose.pose.position = object_pose.position;
    grasp_pose.pose.position.z += gripper_length_ + grasp_height_offset_;
    
    // 姿态：根据物体旋转角调整末端执行器角度
    tf2::Quaternion q;
    q.setRPY(0, 0, object_pose.rotation_angle * M_PI / 180.0);
    grasp_pose.pose.orientation = tf2::toMsg(q);
    
    RCLCPP_INFO(this->get_logger(), 
        "🎯 计算抓取姿态: 目标物体(%.3f, %.3f, %.3f), 机械臂末端(%.3f, %.3f, %.3f)",
        object_pose.position.x, object_pose.position.y, object_pose.position.z,
        grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
    
    RCLCPP_INFO(this->get_logger(), 
        "📏 夹爪偏移: %.2fm (夹爪长度) + %.2fm (安全余量) = %.2fm",
        gripper_length_, grasp_height_offset_, gripper_length_ + grasp_height_offset_);
        
    RCLCPP_INFO(this->get_logger(), 
        "🧭 四元数: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
        grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
        grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);
    return grasp_pose;
}

bool TaskManager::executeGraspSequence(const geometry_msgs::msg::PoseStamped& grasp_pose)
{
    try {
        RCLCPP_INFO(this->get_logger(), "🚀 开始执行抓取序列");
        
        // 步骤1: 移动到初始位置
        RCLCPP_INFO(this->get_logger(), "🏠 步骤1: 移动到初始位置");
        if (!moveToHomePosition()) {
            RCLCPP_ERROR(this->get_logger(), "步骤1失败：无法到达初始位置");
            return false;
        }
        
        // 步骤2: 移动到预抓取位置 (目标物体上方)
        RCLCPP_INFO(this->get_logger(), "⬆️ 步骤2: 移动到预抓取位置");
        auto pre_grasp_pose = grasp_pose;
        pre_grasp_pose.pose.position.z += pre_grasp_height_;
        pose_target_pub_->publish(pre_grasp_pose);
        if (!waitForMotionComplete(8.0)) {
            RCLCPP_ERROR(this->get_logger(), "步骤2失败：无法到达预抓取位置");
            return false;
        }
        
        // 步骤3: 下降到物体位置
        RCLCPP_INFO(this->get_logger(), "⬇️ 步骤3: 下降到物体位置");
        pose_target_pub_->publish(grasp_pose);
        if (!waitForMotionComplete(6.0)) {
            RCLCPP_ERROR(this->get_logger(), "步骤3失败：无法到达物体位置");
            return false;
        }
        
        // 步骤4: 在物体位置等待指定时间
        RCLCPP_INFO(this->get_logger(), "⏱️ 步骤4: 在物体位置等待 %.1f 秒", wait_time_at_object_);
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(wait_time_at_object_ * 1000)));
        RCLCPP_INFO(this->get_logger(), "✅ 等待完成，物体位置任务执行完毕");
        
        auto lift_pose = grasp_pose;
        lift_pose.pose.position.z += 0.05;  // 提升5cm
        pose_target_pub_->publish(lift_pose);
        if (!waitForMotionComplete(4.0)) {
            RCLCPP_WARN(this->get_logger(), "步骤5警告：提升可能未完成");
        }
        
        // 步骤6: 返回初始位置
        RCLCPP_INFO(this->get_logger(), "🏠 步骤6: 返回初始位置");
        if (!moveToHomePosition()) {
            RCLCPP_ERROR(this->get_logger(), "步骤6失败：无法返回初始位置");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "🎉 抓取序列执行完成 (无夹爪控制模式)！");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "💥 抓取序列执行失败: %s", e.what());
        return false;
    }
}

// bool TaskManager::controlGripper(const std::string& command)
// {
//     std_msgs::msg::String gripper_cmd;
//     gripper_cmd.data = command;
//     gripper_command_pub_->publish(gripper_cmd);
    
//     RCLCPP_DEBUG(this->get_logger(), "🤏 发送夹爪命令: %s", command.c_str());
//     return true;
// }

bool TaskManager::waitForMotionComplete(double timeout_seconds)
{
    // 简化版本：使用时间延迟等待
    // 实际应用中应该监听机器人状态来确定运动是否完成
    RCLCPP_DEBUG(this->get_logger(), "⏱️ 等待运动完成... (%.1fs)", timeout_seconds);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(timeout_seconds * 1000)));
    return true;
}

void TaskManager::publishStatus(uint8_t status, const std::string& message)
{
    robot_task::msg::TaskStatus status_msg;
    status_msg.object_name = current_target_object_;
    status_msg.status = status;
    status_msg.message = message;
    status_msg.stamp = this->now();
    
    status_pub_->publish(status_msg);
    
    // 状态映射
    std::string status_str;
    switch (status) {
        case 0: status_str = "IDLE"; break;
        case 1: status_str = "SEARCHING"; break;
        case 2: status_str = "EXECUTING"; break;
        case 3: status_str = "COMPLETED"; break;
        case 4: status_str = "FAILED"; break;
        default: status_str = "UNKNOWN"; break;
    }
    
    RCLCPP_INFO(this->get_logger(), "📊 状态更新 [%s]: %s", status_str.c_str(), message.c_str());
}

} // namespace robot_task