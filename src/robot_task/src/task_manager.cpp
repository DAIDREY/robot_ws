#include "task_manager.hpp"

namespace robot_task
{

TaskManager::TaskManager() : Node("task_manager"), 
    object_detected_(false), task_active_(false), 
    current_robot_state_("UNKNOWN"), grasp_thread_running_(false),
    robot_ready_(false)
{
    initializeStatusSubscription();
    this->declare_parameter("use_motion_planner", false);         // 是否使用motion_planner
    this->declare_parameter("approach_distance", 0.1);          // 接近距离
    this->declare_parameter("retreat_distance", 0.05);          // 后退距离
    this->declare_parameter("max_trajectory_points", 100);      // 最大轨迹点数
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
    use_motion_planner_ = this->get_parameter("use_motion_planner").as_bool();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    retreat_distance_ = this->get_parameter("retreat_distance").as_double();
    max_trajectory_points_ = this->get_parameter("max_trajectory_points").as_int();
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
    
    if (use_motion_planner_) {
        motion_planner_client_ = this->create_client<motion_planner::srv::PlanGraspSequence>(
            "/plan_grasp_sequence");
        
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory", 10);
        
        RCLCPP_INFO(this->get_logger(), "Motion Planner 集成已启用");
    } else {
        RCLCPP_INFO(this->get_logger(), "使用传统姿态控制模式");
    }

    RCLCPP_INFO(this->get_logger(), "Task Manager 已启动，等待抓取请求...");
    RCLCPP_INFO(this->get_logger(), "服务接口: /grasp_object");
    RCLCPP_INFO(this->get_logger(), "监听物体姿态: /robot_visioner/object_pose");
    RCLCPP_INFO(this->get_logger(), "发布控制指令: /pose_target, /gripper_command");
    
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
    RCLCPP_DEBUG(this->get_logger(), "移动到初始位置...");
    
    // 显示当前机器人状态
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        RCLCPP_DEBUG(this->get_logger(), "发送指令前状态: %s", current_robot_state_.c_str());
    }
    
    auto home_pose = getHomePosition();
    pose_target_pub_->publish(home_pose);
    
    RCLCPP_DEBUG(this->get_logger(), "初始位置指令已发送");
    
    // 使用状态等待
    if (!waitForRobotReady(30.0)) {  // 增加超时时间到15秒
        RCLCPP_ERROR(this->get_logger(), "移动到初始位置失败：超时");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "已到达初始位置");
    return true;
}

void TaskManager::graspObjectCallback(
    const std::shared_ptr<robot_task::srv::GraspObject::Request> request,
    std::shared_ptr<robot_task::srv::GraspObject::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到抓取请求: %s", request->object_name.c_str());
    
    // 检查是否有任务正在进行
    if (task_active_) {
        response->success = false;
        response->message = "任务正在进行中，请稍后再试";
        RCLCPP_WARN(this->get_logger(), "任务正在进行中，拒绝新请求");
        return;
    }
    
    // 检查是否有抓取线程在运行
    if (grasp_thread_running_) {
        response->success = false;
        response->message = "抓取线程正在运行，请稍后再试";
        RCLCPP_WARN(this->get_logger(), "抓取线程正在运行，拒绝新请求");
        return;
    }
    
    // 立即返回响应，表示请求已接受
    response->success = true;
    response->message = "抓取请求已接受，正在异步执行";
    
    // 启动异步抓取线程
    if (grasp_thread_.joinable()) {
        grasp_thread_.join();  // 等待之前的线程结束
    }
    
    grasp_thread_running_ = true;
    grasp_thread_ = std::thread(&TaskManager::executeGraspTask, this, request->object_name);
    
    RCLCPP_DEBUG(this->get_logger(), "异步抓取线程已启动");
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
            RCLCPP_DEBUG(this->get_logger(), 
                "收到物体姿态: %s at (%.3f, %.3f, %.3f), 角度: %.1f°, 置信度: %.2f",
                msg->object_name.c_str(),
                msg->position.x, msg->position.y, msg->position.z,
                msg->rotation_angle, msg->confidence);
        }
            
        // 如果正在执行任务且是目标物体，记录日志
        if (task_active_ && msg->object_name == current_target_object_) {
            RCLCPP_DEBUG(this->get_logger(), "检测到目标物体: %s", msg->object_name.c_str());
        }
    }
}

bool TaskManager::waitForObject(const std::string& object_name, robot_task::msg::ObjectPose& pose)
{
    // 这个函数现在只是简单的检查，不阻塞
    if (object_detected_ && latest_object_pose_.object_name == object_name) {
        auto pose_time = rclcpp::Time(latest_object_pose_.stamp);
        auto current_time = this->now();
        
        if ((current_time - pose_time).seconds() < 3.0) {
            pose = latest_object_pose_;
            return true;
        }
    }
    return false;
}

geometry_msgs::msg::PoseStamped TaskManager::calculateGraspPose(const robot_task::msg::ObjectPose& object_pose)
{
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = "base_link";
    grasp_pose.header.stamp = this->now();
    
    // 设置位置（保持原有逻辑不变）
    grasp_pose.pose.position = object_pose.position;
    grasp_pose.pose.position.z += gripper_length_ + grasp_height_offset_;
    
    // 获取物体角度（已经过正确的坐标变换）
    double object_angle_deg = static_cast<double>(object_pose.rotation_angle);
    
    // 计算垂直抓取角度：简单地加90度
    double grasp_angle_deg = object_angle_deg + 90.0;
    
    // 角度标准化到[-90, 90]范围（利用夹爪对称性）
    while (grasp_angle_deg > 90.0) {
        grasp_angle_deg -= 180.0;  // 利用180度对称性
    }
    while (grasp_angle_deg <= -90.0) {
        grasp_angle_deg += 180.0;  // 利用180度对称性
    }
    
    // 转换为弧度用于姿态计算
    double grasp_angle_rad = grasp_angle_deg * M_PI / 180.0;
    
    // 设置机械臂末端执行器的目标姿态
    tf2::Quaternion q;
    q.setRPY(0, 0, grasp_angle_rad);  // Roll=0, Pitch=0, Yaw=抓取角度
    grasp_pose.pose.orientation = tf2::toMsg(q);
    
    // 输出清晰的调试信息，帮助理解抓取策略
    RCLCPP_INFO(this->get_logger(), 
        "📐 抓取策略: 物体朝向 %.1f° → 垂直抓取 %.1f° (%.3f弧度)",
        object_angle_deg, grasp_angle_deg, grasp_angle_rad);
    
    return grasp_pose;
}

void TaskManager::executeGraspTask(std::string object_name)
{
    try {
        // 开始任务
        task_active_ = true;
        current_target_object_ = object_name;
        
        publishStatus(1, "正在搜索物体: " + object_name);
        
        // 等待物体出现
        robot_task::msg::ObjectPose object_pose;
        if (!waitForObjectNonBlocking(object_name, object_pose, 15.0)) {
            publishStatus(4, "搜索失败 - 未找到物体");
            task_active_ = false;
            grasp_thread_running_ = false;
            RCLCPP_ERROR(this->get_logger(), "未找到目标物体: %s", object_name.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "找到物体: %s at (%.3f, %.3f, %.3f), 角度: %.1f°",
                   object_pose.object_name.c_str(),
                   object_pose.position.x, object_pose.position.y, object_pose.position.z,
                   object_pose.rotation_angle);
        
        publishStatus(2, "开始执行抓取");
        
        // 计算抓取姿态
        auto grasp_pose = calculateGraspPose(object_pose);
        
        // 根据配置选择执行方式
        bool success = false;
        if (use_motion_planner_) {
            RCLCPP_INFO(this->get_logger(), "使用Motion Planner模式");
            success = executeGraspSequenceWithMotionPlanner(grasp_pose);
        } else {
            RCLCPP_INFO(this->get_logger(), "使用传统姿态控制模式");
            success = executeGraspSequence(grasp_pose);
        }
        
        if (success) {
            publishStatus(3, "抓取完成");
            RCLCPP_INFO(this->get_logger(), "抓取任务成功完成!");
        } else {
            publishStatus(4, "抓取执行失败");
            RCLCPP_ERROR(this->get_logger(), "抓取执行失败");
        }
        
    } catch (const std::exception& e) {
        publishStatus(4, "发生异常错误");
        RCLCPP_ERROR(this->get_logger(), "抓取过程异常: %s", e.what());
    }
    
    // 任务结束
    task_active_ = false;
    grasp_thread_running_ = false;
    RCLCPP_INFO(this->get_logger(), "抓取线程结束");
}

bool TaskManager::executeGraspSequence(const geometry_msgs::msg::PoseStamped& grasp_pose)
{
    try {
        RCLCPP_INFO(this->get_logger(), "开始执行抓取序列");
        
        // 显示当前机器人状态
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            RCLCPP_INFO(this->get_logger(), "抓取开始前状态: %s", current_robot_state_.c_str());
        }
        
        // 步骤1: 移动到初始位置
        RCLCPP_INFO(this->get_logger(), "步骤1: 移动到初始位置");
        if (!moveToHomePosition()) {
            RCLCPP_ERROR(this->get_logger(), "步骤1失败：无法到达初始位置");
            return false;
        }
        
        // 步骤2: 移动到预抓取位置 (目标物体上方)
        RCLCPP_INFO(this->get_logger(), "⬆步骤2: 移动到预抓取位置");
        auto pre_grasp_pose = grasp_pose;
        pre_grasp_pose.pose.position.z += pre_grasp_height_;
        
        RCLCPP_INFO(this->get_logger(), "预抓取位置: (%.3f, %.3f, %.3f)", 
                   pre_grasp_pose.pose.position.x, 
                   pre_grasp_pose.pose.position.y, 
                   pre_grasp_pose.pose.position.z);
        
        pose_target_pub_->publish(pre_grasp_pose);
        RCLCPP_INFO(this->get_logger(), "预抓取位置指令已发送");
        
        if (!waitForRobotReady(30.0)) {
            RCLCPP_ERROR(this->get_logger(), "步骤2失败：无法到达预抓取位置");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "步骤2: 已到达预抓取位置");
        
        // 步骤3: 下降到物体位置
        RCLCPP_INFO(this->get_logger(), "⬇步骤3: 下降到物体位置");
        RCLCPP_INFO(this->get_logger(), "抓取位置: (%.3f, %.3f, %.3f)", 
                   grasp_pose.pose.position.x, 
                   grasp_pose.pose.position.y, 
                   grasp_pose.pose.position.z);
        
        pose_target_pub_->publish(grasp_pose);
        RCLCPP_INFO(this->get_logger(), "抓取位置指令已发送");
        
        if (!waitForRobotReady(30.0)) {
            RCLCPP_ERROR(this->get_logger(), "步骤3失败：无法到达物体位置");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "步骤3: 已到达物体位置");
        
        // 步骤4: 在物体位置等待指定时间
        RCLCPP_INFO(this->get_logger(), "⏱步骤4: 在物体位置等待 %.1f 秒", wait_time_at_object_);
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(wait_time_at_object_ * 1000)));
        RCLCPP_INFO(this->get_logger(), "步骤4: 等待完成，物体位置任务执行完毕");
        
        // 步骤5: 提升
        RCLCPP_INFO(this->get_logger(), "⬆步骤5: 提升物体");
        auto lift_pose = grasp_pose;
        lift_pose.pose.position.z += 0.05;  // 提升5cm
        
        RCLCPP_INFO(this->get_logger(), "提升位置: (%.3f, %.3f, %.3f)", 
                   lift_pose.pose.position.x, 
                   lift_pose.pose.position.y, 
                   lift_pose.pose.position.z);
        
        pose_target_pub_->publish(lift_pose);
        RCLCPP_INFO(this->get_logger(), "提升位置指令已发送");
        
        if (!waitForRobotReady(30.0)) {
            RCLCPP_WARN(this->get_logger(), "步骤5警告：提升可能未完成");
        } else {
            RCLCPP_INFO(this->get_logger(), "步骤5: 提升完成");
        }
        
        // 步骤6: 返回初始位置
        RCLCPP_INFO(this->get_logger(), "步骤6: 返回初始位置");
        if (!moveToHomePosition()) {
            RCLCPP_ERROR(this->get_logger(), "步骤6失败：无法返回初始位置");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "抓取序列执行完成 (状态驱动模式)！");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "抓取序列执行失败: %s", e.what());
        return false;
    }
}

bool TaskManager::waitForObjectNonBlocking(const std::string& object_name, robot_task::msg::ObjectPose& pose, double timeout_seconds)
{
    RCLCPP_INFO(this->get_logger(), "等待物体出现: %s (超时: %.1fs)", object_name.c_str(), timeout_seconds);
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double>(timeout_seconds);
    
    // 重置检测标志
    object_detected_ = false;
    
    while (grasp_thread_running_) {
        // 检查超时
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout_duration) {
            RCLCPP_WARN(this->get_logger(), "等待物体超时: %s", object_name.c_str());
            return false;
        }
        
        // 检查是否检测到物体
        if (object_detected_ && latest_object_pose_.object_name == object_name) {
            // 检查数据是否足够新
            auto pose_time = rclcpp::Time(latest_object_pose_.stamp);
            auto current_time = std::chrono::steady_clock::now();
            auto pose_age = std::chrono::duration<double>(
                current_time.time_since_epoch()).count() - pose_time.seconds();
            
            if (pose_age < 3.0) {
                pose = latest_object_pose_;
                RCLCPP_INFO(this->get_logger(), "找到物体: %s", object_name.c_str());
                return true;
            } else {
                RCLCPP_DEBUG(this->get_logger(), "物体数据已过期(%.1fs)，继续等待...", pose_age);
                object_detected_ = false; // 重置标志，等待新数据
            }
        }
        
        // 添加一些调试信息
        if (object_detected_) {
            RCLCPP_DEBUG(this->get_logger(), "检测到物体但不是目标: %s (目标: %s)", 
                        latest_object_pose_.object_name.c_str(), object_name.c_str());
        }
        
        // 短暂休眠，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 每2秒显示一次等待状态
        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 2) {
            double remaining = timeout_seconds - std::chrono::duration<double>(elapsed).count();
            RCLCPP_DEBUG(this->get_logger(), "继续等待物体: %s (剩余: %.1fs)", 
                       object_name.c_str(), remaining);
            last_log_time = now;
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "等待物体被中断: %s", object_name.c_str());
    return false;
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
    // 直接使用基于状态的等待
    return waitForRobotReady(timeout_seconds);
}

void TaskManager::initializeStatusSubscription()
{
    robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_status", 10,
        std::bind(&TaskManager::robotStatusCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "已订阅机器人状态: /robot_status");
}

void TaskManager::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    
    std::string new_state = parseRobotState(msg->data);
    
    if (new_state != current_robot_state_) {
        RCLCPP_DEBUG(this->get_logger(), "机器人状态变化: %s -> %s", 
                    current_robot_state_.c_str(), new_state.c_str());
        
        current_robot_state_ = new_state;
        
        // 更新ready标志
        robot_ready_ = (new_state == "READY");
        
        // 通知等待的线程
        state_change_cv_.notify_all();
    }
}

std::string TaskManager::parseRobotState(const std::string& status_message)
{
    // 解析格式: "State: READY, PWM: 1500, XYZ: (...)"
    size_t state_pos = status_message.find("State: ");
    if (state_pos != std::string::npos) {
        size_t start = state_pos + 7; // "State: "的长度
        size_t end = status_message.find(",", start);
        if (end != std::string::npos) {
            return status_message.substr(start, end - start);
        }
    }
    return "UNKNOWN";
}

bool TaskManager::waitForRobotReady(double timeout_seconds)
{
    RCLCPP_DEBUG(this->get_logger(), "等待机器人运动完成... (超时: %.1fs)", timeout_seconds);
    
    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double>(timeout_seconds);
    
    // 阶段1: 先等待一小段时间，让指令生效
    RCLCPP_DEBUG(this->get_logger(), "等待指令生效...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 500ms让指令生效
    
    // 阶段2: 等待开始运动（状态变为MOVING）或直接就绪
    bool started_moving = false;
    auto phase1_timeout = std::chrono::seconds(3);  // 最多等3秒开始运动
    auto phase1_start = std::chrono::steady_clock::now();
    
    while (std::chrono::steady_clock::now() - phase1_start < phase1_timeout) {
        if (current_robot_state_ == "MOVING") {
            started_moving = true;
            RCLCPP_DEBUG(this->get_logger(), "检测到机器人开始运动");
            break;
        } else if (current_robot_state_ == "READY") {
            // 如果已经是READY，可能指令很快完成了，再等待一下确认
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 等待状态变化
        state_change_cv_.wait_for(lock, std::chrono::milliseconds(100));
        
        // 检查总超时
        auto total_elapsed = std::chrono::steady_clock::now() - start_time;
        if (total_elapsed >= timeout_duration) {
            RCLCPP_WARN(this->get_logger(), "等待运动开始超时! 当前状态: %s", 
                       current_robot_state_.c_str());
            return false;
        }
    }
    
    // 阶段3: 如果检测到运动，等待运动完成（状态变为READY）
    if (started_moving) {
        RCLCPP_DEBUG(this->get_logger(), "机器人正在运动，等待完成...");
        
        while (current_robot_state_ == "MOVING") {
            // 等待状态变化
            if (state_change_cv_.wait_for(lock, std::chrono::milliseconds(200)) == std::cv_status::timeout) {
                // 显示等待状态（每2秒）
                static auto last_log_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 2) {
                    auto elapsed = std::chrono::duration<double>(now - start_time).count();
                    RCLCPP_DEBUG(this->get_logger(), "运动中... 已用时: %.1fs", elapsed);
                    last_log_time = now;
                }
            }
            
            // 检查总超时
            auto total_elapsed = std::chrono::steady_clock::now() - start_time;
            if (total_elapsed >= timeout_duration) {
                RCLCPP_WARN(this->get_logger(), "等待运动完成超时! 当前状态: %s", 
                           current_robot_state_.c_str());
                return false;
            }
        }
    } else {
        // 没有检测到运动，可能指令很快完成，添加最小等待时间
        RCLCPP_DEBUG(this->get_logger(), "未检测到运动状态变化，添加最小等待时间");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // 最少等1.5秒
    }
    
    // 最终确认是否就绪
    if (current_robot_state_ == "READY") {
        auto total_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_DEBUG(this->get_logger(), "机器人运动完成! 总用时: %.2fs", total_time);
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "机器人状态异常: %s", current_robot_state_.c_str());
        return false;
    }
}

bool TaskManager::planGraspWithMotionPlanner(const geometry_msgs::msg::Pose& target_pose)
{
    if (!motion_planner_client_) {
        RCLCPP_ERROR(this->get_logger(), "Motion planner客户端未初始化");
        return false;
    }
    
    // 等待motion_planner服务可用
    if (!waitForPlannerReady(10.0)) {
        RCLCPP_ERROR(this->get_logger(), "Motion planner服务不可用");
        return false;
    }
    
    // 创建规划请求
    auto request = std::make_shared<motion_planner::srv::PlanGraspSequence::Request>();
    request->target_pose = target_pose;
    request->approach_distance = approach_distance_;
    request->retreat_distance = retreat_distance_;
    request->eef_step = 0.005;  // 5mm步长
    request->jump_threshold = 0.0;
    request->max_trajectory_points = max_trajectory_points_;
    request->use_cartesian_path = true;
    
    RCLCPP_INFO(this->get_logger(), "📞 调用motion_planner规划抓取序列...");
    
    // 调用服务
    auto future = motion_planner_client_->async_send_request(request);
    
    // 等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "调用motion_planner服务失败");
        return false;
    }
    
    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Motion_planner规划失败: %s", response->message.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Motion_planner规划成功，总轨迹点数: %d", response->total_points);
    
    // 依次执行三段轨迹：接近 -> 抓取 -> 后退
    
    // 1. 执行接近轨迹
    RCLCPP_INFO(this->get_logger(), "执行接近轨迹 (%zu点)", response->approach_trajectory.points.size());
    if (!sendTrajectoryToRobotDriver(response->approach_trajectory)) {
        return false;
    }
    
    // 等待接近运动完成
    double approach_time = 0.0;
    if (!response->approach_trajectory.points.empty()) {
        auto last_point = response->approach_trajectory.points.back();
        approach_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(approach_time + 1.0) 
    ));
    
    // 2. 执行抓取轨迹
    RCLCPP_INFO(this->get_logger(), "执行抓取轨迹 (%zu点)", response->grasp_trajectory.points.size());
    if (!sendTrajectoryToRobotDriver(response->grasp_trajectory)) {
        return false;
    }
    
    // 等待抓取运动完成
    double grasp_time = 0.0;
    if (!response->grasp_trajectory.points.empty()) {
        auto last_point = response->grasp_trajectory.points.back();
        grasp_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(grasp_time + 0.5)
    ));
    
    // 3. 执行夹爪闭合（这里需要你实现具体的夹爪控制）
    RCLCPP_INFO(this->get_logger(), "执行夹爪闭合");
    // TODO: 添加夹爪控制代码
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // 4. 执行后退轨迹
    RCLCPP_INFO(this->get_logger(), "⬆执行后退轨迹 (%zu点)", response->retreat_trajectory.points.size());
    if (!sendTrajectoryToRobotDriver(response->retreat_trajectory)) {
        return false;
    }
    
    // 等待后退运动完成
    double retreat_time = 0.0;
    if (!response->retreat_trajectory.points.empty()) {
        auto last_point = response->retreat_trajectory.points.back();
        retreat_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
    }
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(retreat_time + 0.5)
    ));
    
    RCLCPP_INFO(this->get_logger(), "Motion_planner抓取序列执行完成");
    return true;
}

bool TaskManager::sendTrajectoryToRobotDriver(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (!trajectory_pub_) {
        RCLCPP_ERROR(this->get_logger(), "轨迹发布者未初始化");
        return false;
    }
    
    if (trajectory.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "轨迹为空，跳过发送");
        return true;
    }
    
    RCLCPP_INFO(this->get_logger(), "发送轨迹到robot_driver，点数: %zu", trajectory.points.size());
    
    // 设置时间戳
    auto traj_msg = trajectory;
    traj_msg.header.stamp = this->now();
    traj_msg.header.frame_id = "base_link";
    
    // 发布轨迹
    trajectory_pub_->publish(traj_msg);
    
    return true;
}

bool TaskManager::waitForPlannerReady(double timeout_seconds)
{
    RCLCPP_DEBUG(this->get_logger(), "等待motion_planner服务...");
    
    return motion_planner_client_->wait_for_service(std::chrono::duration<double>(timeout_seconds));
}

bool TaskManager::executeGraspSequenceWithMotionPlanner(const geometry_msgs::msg::PoseStamped& grasp_pose)
{
    // 1. 创建服务请求
    auto request = std::make_shared<motion_planner::srv::PlanGraspSequence::Request>();
    request->target_pose = grasp_pose.pose;  // 修正：使用pose成员

    // 2. 检查服务是否可用
    if (!motion_planner_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Motion Planner 服务不可用");
        return false;
    }

    // 3. 发送异步请求
    auto future = motion_planner_client_->async_send_request(request);

    // 4. 等待响应（使用节点已关联的执行器上下文）
    auto result = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        future,
        std::chrono::seconds(10)
    );

    // 5. 处理响应结果
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "调用 Motion Planner 服务超时或失败");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Motion Planner 规划失败: %s", response->message.c_str());
        return false;
    }

    // 6. 发布规划的轨迹（修正成员名称）
    trajectory_pub_->publish(response->grasp_trajectory);
    return waitForMotionComplete();
}

void TaskManager::printRobotStatus()
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    RCLCPP_INFO(this->get_logger(), "📊 当前机器人状态: %s (Ready: %s)", 
               current_robot_state_.c_str(), robot_ready_ ? "是" : "否");
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

TaskManager::~TaskManager()
{
    // 停止抓取线程
    grasp_thread_running_ = false;
    if (grasp_thread_.joinable()) {
        grasp_thread_.join();
    }
}

} // namespace robot_task