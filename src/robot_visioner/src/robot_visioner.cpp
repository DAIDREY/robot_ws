// src/robot_visioner.cpp
#include "robot_visioner.hpp"
#include <chrono>

namespace robot_visioner
{

RobotVisioner::RobotVisioner() 
    : Node("robot_visioner")
    , camera_info_received_(false)
    , processed_frames_(0)
{
    // 初始化参数
    initializeParameters();
    
    // 获取话题名称参数
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string mask_topic = this->get_parameter("mask_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    rgb_topic_ = this->get_parameter("rgb_topic").as_string();
    
    // 订阅者
    rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, 10, 
        std::bind(&RobotVisioner::rgbImageCallback, this, std::placeholders::_1));
        
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, 10, 
        std::bind(&RobotVisioner::depthCallback, this, std::placeholders::_1));
        
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        mask_topic, 10,
        std::bind(&RobotVisioner::maskCallback, this, std::placeholders::_1));
        
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&RobotVisioner::cameraInfoCallback, this, std::placeholders::_1));
    
    // 新增：订阅YOLO检测信息
    detection_info_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/yolo/detection_info", 10,
        std::bind(&RobotVisioner::detectionInfoCallback, this, std::placeholders::_1));
    
    // 发布者
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, 10);
        
    center_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/robot_visioner/center_point", 10);
        
    center_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/robot_visioner/center_marker", 10);
        
    cluster_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/robot_visioner/cluster_markers", 10);
    
    // 新增：发布ObjectPose给robot_task
    object_pose_pub_ = this->create_publisher<robot_task::msg::ObjectPose>(
        "/robot_visioner/object_pose", 10);
    
    // 初始化检测信息
    latest_detection_.valid = false;
    
    // 输出节点信息
    logNodeInfo();
}

void RobotVisioner::initializeParameters()
{
    // 声明参数
    this->declare_parameter("rgb_topic", "/camera/color/image_raw");
    this->declare_parameter("depth_topic", "/camera/depth/image_raw");
    this->declare_parameter("mask_topic", "/yolo/mask");
    this->declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    this->declare_parameter("output_topic", "/extracted_pointcloud");
    this->declare_parameter("frame_id", "camera_link");
    
    // 处理参数
    this->declare_parameter("depth_scale", 1000.0);
    this->declare_parameter("mask_threshold", 128);
    this->declare_parameter("sync_tolerance", 0.05);
    this->declare_parameter("enable_rgb_color", true);
    
    // 聚类参数
    this->declare_parameter("enable_clustering", true);
    this->declare_parameter("cluster_tolerance", 0.02);
    this->declare_parameter("min_cluster_size", 30);
    this->declare_parameter("max_cluster_size", 25000);
    
    // 过滤参数
    this->declare_parameter("enable_voxel_filter", true);
    this->declare_parameter("voxel_leaf_size", 0.008);
    this->declare_parameter("enable_outlier_filter", true);
    this->declare_parameter("outlier_mean_k", 30);
    this->declare_parameter("outlier_stddev_mul", 1.0);
    
    // 工作空间参数
    this->declare_parameter("workspace_x_min", -2.0);
    this->declare_parameter("workspace_x_max", 2.0);
    this->declare_parameter("workspace_y_min", -2.0);
    this->declare_parameter("workspace_y_max", 2.0);
    this->declare_parameter("workspace_z_min", 0.1);
    this->declare_parameter("workspace_z_max", 3.0);
    
    // 可视化参数
    this->declare_parameter("marker_scale", 0.01);
    
    this->declare_parameter("target_frame", "base_link");        // 目标坐标系
    this->declare_parameter("source_frame", "camera_link");  // 源坐标系

    this->declare_parameter("position_offset_x", 0.0);    // X方向偏移 (前后方向)
    this->declare_parameter("position_offset_y", 0.0);    // Y方向偏移 (左右方向)
    this->declare_parameter("position_offset_z", 0.0);    // Z方向偏移 (上下方向)
    this->declare_parameter("enable_position_offset", true);  // 是否启用偏移

    // 获取参数值
    depth_scale_ = this->get_parameter("depth_scale").as_double();
    mask_threshold_ = this->get_parameter("mask_threshold").as_int();
    sync_tolerance_ = this->get_parameter("sync_tolerance").as_double();
    enable_rgb_color_ = this->get_parameter("enable_rgb_color").as_bool();
    
    enable_clustering_ = this->get_parameter("enable_clustering").as_bool();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    
    enable_voxel_filter_ = this->get_parameter("enable_voxel_filter").as_bool();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    enable_outlier_filter_ = this->get_parameter("enable_outlier_filter").as_bool();
    outlier_mean_k_ = this->get_parameter("outlier_mean_k").as_int();
    outlier_stddev_mul_ = this->get_parameter("outlier_stddev_mul").as_double();
    
    workspace_x_min_ = this->get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = this->get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = this->get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = this->get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = this->get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = this->get_parameter("workspace_z_max").as_double();
    
    marker_scale_ = this->get_parameter("marker_scale").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    source_frame_ = this->get_parameter("source_frame").as_string();

    position_offset_x_ = this->get_parameter("position_offset_x").as_double();
    position_offset_y_ = this->get_parameter("position_offset_y").as_double();
    position_offset_z_ = this->get_parameter("position_offset_z").as_double();
    enable_position_offset_ = this->get_parameter("enable_position_offset").as_bool();
    
}

void RobotVisioner::logNodeInfo()
{
    RCLCPP_INFO(this->get_logger(), "🤖 Robot Visioner 节点已启动");
    RCLCPP_INFO(this->get_logger(), "📊 配置信息:");
    RCLCPP_INFO(this->get_logger(), "   RGB话题: %s", rgb_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "   深度话题: %s", this->get_parameter("depth_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "   Mask话题: %s", this->get_parameter("mask_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "   检测信息话题: /yolo/detection_info");
    RCLCPP_INFO(this->get_logger(), "   坐标系: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "📤 发布话题:");
    RCLCPP_INFO(this->get_logger(), "   点云: %s", this->get_parameter("output_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "   中心点: /robot_visioner/center_point");
    RCLCPP_INFO(this->get_logger(), "   物体姿态: /robot_visioner/object_pose");
    RCLCPP_INFO(this->get_logger(), "🔧 RGB彩色点云: %s", enable_rgb_color_ ? "启用" : "禁用");
    RCLCPP_INFO(this->get_logger(), "🔧 聚类分析: %s", enable_clustering_ ? "启用" : "禁用");
    RCLCPP_INFO(this->get_logger(), "🔧 体素过滤: %s", enable_voxel_filter_ ? "启用" : "禁用");
    RCLCPP_INFO(this->get_logger(), "   源坐标系: %s", source_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "   目标坐标系: %s", target_frame_.c_str());
    
    RCLCPP_INFO(this->get_logger(), "📤 发布话题:");
    RCLCPP_INFO(this->get_logger(), "   中心点(base_link坐标系): /robot_visioner/center_point");
    RCLCPP_INFO(this->get_logger(), "   物体姿态(base_link坐标系): /robot_visioner/object_pose");
}

void RobotVisioner::rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    rgb_image_ = msg;
    tryExtractPointCloud();
}

void RobotVisioner::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    depth_image_ = msg;
    tryExtractPointCloud();
}

void RobotVisioner::maskCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    mask_image_ = msg;
    tryExtractPointCloud();
}

void RobotVisioner::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_info_ = msg;
    
    // 提取相机内参
    fx_ = msg->k[0];  // K[0]
    fy_ = msg->k[4];  // K[4] 
    cx_ = msg->k[2];  // K[2]
    cy_ = msg->k[5];  // K[5]
    
    camera_info_received_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "相机内参: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                     fx_, fy_, cx_, cy_);
}

void RobotVisioner::detectionInfoCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(detection_mutex_);
    
    DetectionInfo info;
    if (parseDetectionInfo(msg->data, info)) {
        info.timestamp = this->now();
        latest_detection_ = info;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "收到检测信息: %s, 角度: %.1f°, 置信度: %.2f",
            info.object_name.c_str(), info.rotation_angle, info.confidence);
    } else {
        RCLCPP_WARN(this->get_logger(), "解析检测信息失败: %s", msg->data.c_str());
        latest_detection_.valid = false;
    }
}

bool RobotVisioner::parseDetectionInfo(const std::string& json_str, DetectionInfo& info)
{
    try {
        // 更健壮的JSON解析
        // 查找object_name
        size_t name_start = json_str.find("\"object_name\": \"");
        if (name_start == std::string::npos) {
            name_start = json_str.find("\"object_name\":\"");
        }
        if (name_start != std::string::npos) {
            name_start = json_str.find("\"", name_start + 14) + 1;
            size_t name_end = json_str.find("\"", name_start);
            info.object_name = json_str.substr(name_start, name_end - name_start);
        }
        
        // 查找center_x
        size_t x_start = json_str.find("\"center_x\": ");
        if (x_start == std::string::npos) {
            x_start = json_str.find("\"center_x\":");
        }
        if (x_start != std::string::npos) {
            x_start = json_str.find(":", x_start) + 1;
            // 跳过空格
            while (x_start < json_str.length() && std::isspace(json_str[x_start])) {
                x_start++;
            }
            size_t x_end = json_str.find(",", x_start);
            info.center_x = std::stod(json_str.substr(x_start, x_end - x_start));
        }
        
        // 查找center_y
        size_t y_start = json_str.find("\"center_y\": ");
        if (y_start == std::string::npos) {
            y_start = json_str.find("\"center_y\":");
        }
        if (y_start != std::string::npos) {
            y_start = json_str.find(":", y_start) + 1;
            // 跳过空格
            while (y_start < json_str.length() && std::isspace(json_str[y_start])) {
                y_start++;
            }
            size_t y_end = json_str.find(",", y_start);
            info.center_y = std::stod(json_str.substr(y_start, y_end - y_start));
        }
        
        // 查找rotation_angle
        size_t angle_start = json_str.find("\"rotation_angle\": ");
        if (angle_start == std::string::npos) {
            angle_start = json_str.find("\"rotation_angle\":");
        }
        if (angle_start != std::string::npos) {
            angle_start = json_str.find(":", angle_start) + 1;
            // 跳过空格
            while (angle_start < json_str.length() && std::isspace(json_str[angle_start])) {
                angle_start++;
            }
            size_t angle_end = json_str.find(",", angle_start);
            info.rotation_angle = std::stod(json_str.substr(angle_start, angle_end - angle_start));
        }
        
        // 查找confidence
        size_t conf_start = json_str.find("\"confidence\": ");
        if (conf_start == std::string::npos) {
            conf_start = json_str.find("\"confidence\":");
        }
        if (conf_start != std::string::npos) {
            conf_start = json_str.find(":", conf_start) + 1;
            // 跳过空格
            while (conf_start < json_str.length() && std::isspace(json_str[conf_start])) {
                conf_start++;
            }
            size_t conf_end = json_str.find(",", conf_start);
            if (conf_end == std::string::npos) {
                conf_end = json_str.find("}", conf_start);
            }
            info.confidence = std::stod(json_str.substr(conf_start, conf_end - conf_start));
        }
        
        info.valid = true;
        RCLCPP_DEBUG(this->get_logger(), "解析成功: %s, 角度: %.1f°", 
                    info.object_name.c_str(), info.rotation_angle);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "解析检测信息失败: %s", e.what());
        info.valid = false;
        return false;
    }
}

void RobotVisioner::tryExtractPointCloud()
{
    // 检查是否所有数据都已准备好
    if (!depth_image_ || !mask_image_ || !camera_info_received_) {
        return;
    }
    
    // RGB图像是可选的
    if (enable_rgb_color_ && !rgb_image_) {
        return;
    }
    
    // 检查时间戳同步
    auto depth_time = rclcpp::Time(depth_image_->header.stamp);
    auto mask_time = rclcpp::Time(mask_image_->header.stamp);
    
    if (std::abs((depth_time - mask_time).seconds()) > sync_tolerance_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "深度图和mask时间戳不同步: %.3f秒差异", 
                             std::abs((depth_time - mask_time).seconds()));
        return;
    }
    
    // 如果启用RGB，也检查RGB同步
    if (enable_rgb_color_ && rgb_image_) {
        auto rgb_time = rclcpp::Time(rgb_image_->header.stamp);
        if (std::abs((depth_time - rgb_time).seconds()) > sync_tolerance_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "深度图和RGB时间戳不同步: %.3f秒差异", 
                                 std::abs((depth_time - rgb_time).seconds()));
            return;
        }
    }
    
    extractMaskedPointCloud();
}

void RobotVisioner::extractMaskedPointCloud()
{
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 转换ROS图像到OpenCV格式
        cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image_, "16UC1");
        cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(mask_image_, "mono8");
        
        cv::Mat depth_img = depth_ptr->image;
        cv::Mat mask_img = mask_ptr->image;
        cv::Mat rgb_img;
        
        // 转换RGB图像（如果启用）
        if (enable_rgb_color_ && rgb_image_) {
            try {
                cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_image_, "bgr8");
                rgb_img = rgb_ptr->image;
                
                // 确保RGB和深度图尺寸匹配
                if (rgb_img.size() != depth_img.size()) {
                    cv::resize(rgb_img, rgb_img, depth_img.size());
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "RGB图像转换失败: %s，使用深度着色", e.what());
                rgb_img = cv::Mat();
            }
        }
        
        // 确保尺寸匹配
        if (depth_img.size() != mask_img.size()) {
            cv::resize(mask_img, mask_img, depth_img.size());
        }
        
        // 创建彩色点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 遍历所有像素生成彩色点云
        for (int v = 0; v < depth_img.rows; ++v) {
            for (int u = 0; u < depth_img.cols; ++u) {
                // 检查mask值
                if (mask_img.at<uint8_t>(v, u) < mask_threshold_) {
                    continue;
                }
                
                // 获取深度值
                uint16_t depth_raw = depth_img.at<uint16_t>(v, u);
                if (depth_raw == 0) {
                    continue;  // 无效深度
                }
                
                // 深度值转换
                double depth = static_cast<double>(depth_raw) / depth_scale_;
                
                // 像素坐标转换为3D坐标
                double x = (u - cx_) * depth / fx_;
                double y = (v - cy_) * depth / fy_;
                double z = depth;
                
                // 工作空间过滤
                if (x < workspace_x_min_ || x > workspace_x_max_ ||
                    y < workspace_y_min_ || y > workspace_y_max_ ||
                    z < workspace_z_min_ || z > workspace_z_max_) {
                    continue;
                }
                
                // 创建彩色点
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                
                // 添加颜色信息
                if (enable_rgb_color_ && !rgb_img.empty() && 
                    v < rgb_img.rows && u < rgb_img.cols) {
                    // 使用真实RGB颜色
                    cv::Vec3b color = rgb_img.at<cv::Vec3b>(v, u);
                    point.r = color[2];  // OpenCV是BGR，PCL是RGB
                    point.g = color[1];
                    point.b = color[0];
                } else {
                    // 使用深度着色
                    cv::Vec3b depth_color = getDepthColor(depth, workspace_z_min_, workspace_z_max_);
                    point.r = depth_color[0];
                    point.g = depth_color[1];
                    point.b = depth_color[2];
                }
                
                cloud->points.push_back(point);
            }
        }
        
        // 设置点云属性
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        if (cloud->points.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "未提取到有效点云数据");
            return;
        }
        
        // 应用点云过滤
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = filterPointCloud(cloud);
        
        // 计算中心点
        std::vector<CenterPoint3D> center_points;
        if (enable_clustering_) {
            center_points = extractClusterCentroids(filtered_cloud);
        } else {
            CenterPoint3D single_center = calculateCentroid(filtered_cloud);
            if (single_center.point_count > 0) {
                center_points.push_back(single_center);
            }
        }
        
        // 发布结果
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = depth_image_->header;
        output_msg.header.frame_id = frame_id_;
        pointcloud_pub_->publish(output_msg);
        
        // 发布中心点和物体姿态
        if (!center_points.empty()) {
            publishCenterPoints(center_points, output_msg.header);
            publishCenterMarkers(center_points, output_msg.header);
        }
        
        // 统计信息
        processed_frames_++;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "✅ 处理完成: %zu个彩色点 | %zu个中心点 | 耗时: %ldms | 总帧数: %zu",
                    filtered_cloud->points.size(), center_points.size(), 
                    duration.count(), processed_frames_);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "💥 点云提取失败: %s", e.what());
    }
}

cv::Vec3b RobotVisioner::getDepthColor(double depth, double min_depth, double max_depth)
{
    // 将深度值映射到0-1范围
    double normalized = (depth - min_depth) / (max_depth - min_depth);
    normalized = std::max(0.0, std::min(1.0, normalized));  // 限制在[0,1]
    
    // 创建彩虹色映射
    cv::Vec3b color;
    if (normalized < 0.25) {
        // 蓝色到青色
        double t = normalized / 0.25;
        color[0] = static_cast<uint8_t>(0);           // R
        color[1] = static_cast<uint8_t>(255 * t);     // G  
        color[2] = static_cast<uint8_t>(255);         // B
    } else if (normalized < 0.5) {
        // 青色到绿色
        double t = (normalized - 0.25) / 0.25;
        color[0] = static_cast<uint8_t>(0);           // R
        color[1] = static_cast<uint8_t>(255);         // G
        color[2] = static_cast<uint8_t>(255 * (1-t)); // B
    } else if (normalized < 0.75) {
        // 绿色到黄色
        double t = (normalized - 0.5) / 0.25;
        color[0] = static_cast<uint8_t>(255 * t);     // R
        color[1] = static_cast<uint8_t>(255);         // G
        color[2] = static_cast<uint8_t>(0);           // B
    } else {
        // 黄色到红色
        double t = (normalized - 0.75) / 0.25;
        color[0] = static_cast<uint8_t>(255);         // R
        color[1] = static_cast<uint8_t>(255 * (1-t)); // G
        color[2] = static_cast<uint8_t>(0);           // B
    }
    
    return color;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RobotVisioner::filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = input_cloud;
    
    // 体素滤波
    if (enable_voxel_filter_ && !input_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        voxel_filter.filter(*voxel_filtered);
        filtered_cloud = voxel_filtered;
    }
    
    // 离群点过滤
    if (enable_outlier_filter_ && filtered_cloud->size() > static_cast<size_t>(outlier_mean_k_)) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;
        outlier_filter.setInputCloud(filtered_cloud);
        outlier_filter.setMeanK(outlier_mean_k_);
        outlier_filter.setStddevMulThresh(outlier_stddev_mul_);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        outlier_filter.filter(*outlier_filtered);
        filtered_cloud = outlier_filtered;
    }
    
    return filtered_cloud;
}

CenterPoint3D RobotVisioner::calculateCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    if (cloud->empty()) {
        return CenterPoint3D();
    }
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    
    CenterPoint3D center(centroid[0], centroid[1], centroid[2], 
                        cloud->size(), 1.0);
    
    validateCenterPoint(center);
    return center;
}

std::vector<CenterPoint3D> RobotVisioner::extractClusterCentroids(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    std::vector<CenterPoint3D> centers;
    
    if (cloud->empty()) {
        return centers;
    }
    
    // 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // 计算每个聚类的中心点
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for (const auto& index : indices.indices) {
            cluster_cloud->points.push_back(cloud->points[index]);
        }
        
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = false;
        
        CenterPoint3D center = calculateCentroid(cluster_cloud);
        if (center.point_count > 0) {
            center.confidence = static_cast<double>(center.point_count) / cloud->size();
            centers.push_back(center);
        }
    }
    
    return centers;
}

void RobotVisioner::validateCenterPoint(CenterPoint3D& center)
{
    // 检查中心点是否在工作空间内
    if (center.x < workspace_x_min_ || center.x > workspace_x_max_ ||
        center.y < workspace_y_min_ || center.y > workspace_y_max_ ||
        center.z < workspace_z_min_ || center.z > workspace_z_max_) {
        
        RCLCPP_WARN(this->get_logger(), "中心点超出工作空间: (%.3f, %.3f, %.3f)", 
                   center.x, center.y, center.z);
        center.confidence *= 0.5;  // 降低置信度
    }
}

void RobotVisioner::publishCenterPoints(const std::vector<CenterPoint3D>& centers, 
                                       const std_msgs::msg::Header& header)
{
    if (centers.empty()) return;
    
    // 选择最佳中心点（置信度最高的）
    auto best_center = *std::max_element(centers.begin(), centers.end(),
        [](const CenterPoint3D& a, const CenterPoint3D& b) {
            return a.confidence < b.confidence;
        });
    
    // 创建相机坐标系下的点
    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header = header;
    camera_point.point.x = best_center.x;
    camera_point.point.y = best_center.y;
    camera_point.point.z = best_center.z;
    
    // 转换到base_link坐标系
    geometry_msgs::msg::PointStamped base_link_point;
    if (transformPointToBaseLink(camera_point, base_link_point)) {
        // 发布转换后的点（base_link坐标系）
        center_point_pub_->publish(base_link_point);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "📍 物体中心点 (base_link): x=%.3fm, y=%.3fm, z=%.3fm, 置信度=%.2f",
            base_link_point.point.x, base_link_point.point.y, base_link_point.point.z,
            best_center.confidence);
    } else {
        // 如果变换失败，发布原始的相机坐标系点（向后兼容）
        RCLCPP_WARN(this->get_logger(), "坐标变换失败，发布相机坐标系下的点");
        center_point_pub_->publish(camera_point);
    }
    
    // 发布ObjectPose消息（也转换到base_link）
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        
        if (latest_detection_.valid) {
            auto detection_age = (this->now() - latest_detection_.timestamp).seconds();
            if (detection_age < 3.0) {
                robot_task::msg::ObjectPose object_pose;
                object_pose.object_name = latest_detection_.object_name;
                
                // 使用base_link坐标系的位置
                if (transformPointToBaseLink(camera_point, base_link_point)) {
                    // 获取变换后的原始坐标
                    double raw_x = base_link_point.point.x;
                    double raw_y = base_link_point.point.y;
                    double raw_z = base_link_point.point.z;
                    
                    // 应用位置偏移
                    if (enable_position_offset_) {
                        object_pose.position.x = raw_x + position_offset_x_;
                        object_pose.position.y = raw_y + position_offset_y_;
                        object_pose.position.z = raw_z + position_offset_z_;
                        
                        RCLCPP_DEBUG(this->get_logger(), 
                            "📐 位置偏移: 原始(%.3f,%.3f,%.3f) → 偏移后(%.3f,%.3f,%.3f)",
                            raw_x, raw_y, raw_z,
                            object_pose.position.x, object_pose.position.y, object_pose.position.z);
                        
                        static int offset_log_counter = 0;
                        if (++offset_log_counter % 10 == 0) {
                            RCLCPP_DEBUG(this->get_logger(), 
                                "🎯 位置偏移: (%.3f,%.3f,%.3f)",
                                position_offset_x_, position_offset_y_, position_offset_z_);
                        }
                    } else {
                        object_pose.position.x = raw_x;
                        object_pose.position.y = raw_y;
                        object_pose.position.z = raw_z;
                    }
                    
                    object_pose.stamp = base_link_point.header.stamp;
                } else {
                    // 变换失败时使用相机坐标系
                    object_pose.position.x = best_center.x;
                    object_pose.position.y = best_center.y;
                    object_pose.position.z = best_center.z;
                    object_pose.stamp = header.stamp;
                    
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "⚠️ TF变换失败，跳过位置偏移");
                }
                
                double corrected_rotation_angle = 0.0;

                // 角度变换
                if (transformRotationToBaseLink(latest_detection_.rotation_angle, 
                                              header.frame_id, 
                                              corrected_rotation_angle)) {
                    RCLCPP_DEBUG(this->get_logger(), 
                        "📐 角度变换: %.1f° (相机:%s) -> %.1f° (机械臂:%s)",
                        static_cast<double>(latest_detection_.rotation_angle), 
                        header.frame_id.c_str(),
                        corrected_rotation_angle, 
                        target_frame_.c_str());
                } else {
                    // 变换失败时的备选策略
                    corrected_rotation_angle = static_cast<double>(latest_detection_.rotation_angle);
                    
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "⚠️ 角度变换失败，使用原始角度: %.1f°",
                        corrected_rotation_angle);
                }

                // 角度标准化到[-90, 90]范围
                corrected_rotation_angle = fmod(corrected_rotation_angle, 360.0);
                if (corrected_rotation_angle < 0) {
                    corrected_rotation_angle += 360.0;
                }
                
                // 映射到 -90°~90°
                if (corrected_rotation_angle > 90.0 && corrected_rotation_angle <= 270.0) {
                    corrected_rotation_angle -= 180.0;
                } else if (corrected_rotation_angle > 270.0) {
                    corrected_rotation_angle -= 360.0;
                }
                
                // 边界保护
                if (corrected_rotation_angle > 90.0) corrected_rotation_angle = 90.0;
                if (corrected_rotation_angle < -90.0) corrected_rotation_angle = -90.0;

                object_pose.rotation_angle = static_cast<float>(corrected_rotation_angle);
                object_pose.rotation_angle = static_cast<float>(corrected_rotation_angle);
                object_pose.confidence = static_cast<float>(latest_detection_.confidence * best_center.confidence);
                
                object_pose_pub_->publish(object_pose);
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "📤 发布物体姿态: %s at (%.3f, %.3f, %.3f), 角度: %.1f°, 置信度: %.2f",
                    object_pose.object_name.c_str(),
                    object_pose.position.x, object_pose.position.y, object_pose.position.z,
                    object_pose.rotation_angle, object_pose.confidence);
            }
        }
    }
}

void RobotVisioner::publishCenterMarkers(const std::vector<CenterPoint3D>& centers, 
                                        const std_msgs::msg::Header& header)
{
    // 发布单个最佳中心点标记
    if (!centers.empty()) {
        auto best_center = *std::max_element(centers.begin(), centers.end(),
            [](const CenterPoint3D& a, const CenterPoint3D& b) {
                return a.confidence < b.confidence;
            });
        
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "center_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = best_center.x;
        marker.pose.position.y = best_center.y;
        marker.pose.position.z = best_center.z;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = marker_scale_ * 0.1;  // 更大的标记
        marker.scale.y = marker_scale_ * 0.1;
        marker.scale.z = marker_scale_ * 0.1;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.9;  // 更不透明
        
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        center_marker_pub_->publish(marker);
    }
    
    // 发布所有聚类中心标记
    if (enable_clustering_ && centers.size() > 1) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < centers.size(); ++i) {
            const auto& center = centers[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "cluster_centers";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = center.x;
            marker.pose.position.y = center.y;
            marker.pose.position.z = center.z;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = marker_scale_ * (1.0 + center.confidence);
            marker.scale.y = marker_scale_ * (1.0 + center.confidence);
            marker.scale.z = marker_scale_ * (1.0 + center.confidence);
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;  // 绿色
            marker.color.b = 0.0;
            marker.color.a = std::min(1.0, center.confidence * 2.0);
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            marker_array.markers.push_back(marker);
        }
        
        cluster_markers_pub_->publish(marker_array);
    }
}

bool RobotVisioner::transformPointToBaseLink(const geometry_msgs::msg::PointStamped& point_in,
                                             geometry_msgs::msg::PointStamped& point_out)
{
    try {
        // 等待变换可用（超时时间为1秒）
        if (!tf_buffer_->canTransform(target_frame_, point_in.header.frame_id, 
                                     point_in.header.stamp, rclcpp::Duration::from_seconds(1.0))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "无法获取从 %s 到 %s 的变换", 
                point_in.header.frame_id.c_str(), target_frame_.c_str());
            return false;
        }
        
        // 执行坐标变换
        tf_buffer_->transform(point_in, point_out, target_frame_);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "坐标变换: (%.3f, %.3f, %.3f) %s -> (%.3f, %.3f, %.3f) %s",
            point_in.point.x, point_in.point.y, point_in.point.z, point_in.header.frame_id.c_str(),
            point_out.point.x, point_out.point.y, point_out.point.z, point_out.header.frame_id.c_str());
        
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TF2变换失败: %s", ex.what());
        return false;
    }
}

bool RobotVisioner::transformRotationToBaseLink(double camera_angle_deg, 
                                                const std::string& camera_frame,
                                                double& base_angle_deg)
{
    try {
        // 获取从相机到机械臂坐标系的变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame_,        // 目标: base_link
            camera_frame,         // 源: camera_depth_optical_frame
            tf2::TimePointZero    // 最新变换
        );
        
        // 创建相机坐标系下的方向向量
        double camera_angle_rad = camera_angle_deg * M_PI / 180.0;
        tf2::Vector3 direction_camera(cos(camera_angle_rad), sin(camera_angle_rad), 0);
        
        // 应用旋转变换
        tf2::Quaternion rotation_quat;
        tf2::fromMsg(transform_stamped.transform.rotation, rotation_quat);
        tf2::Matrix3x3 rotation_matrix(rotation_quat);
        tf2::Vector3 direction_base = rotation_matrix * direction_camera;
        
        // 提取变换后的角度
        base_angle_deg = atan2(direction_base.y(), direction_base.x()) * 180.0 / M_PI;
        
        // 添加详细的调试信息
        RCLCPP_DEBUG(this->get_logger(), 
            "🔄 方向向量变换: (%.3f,%.3f) -> (%.3f,%.3f), 角度: %.1f° -> %.1f°",
            direction_camera.x(), direction_camera.y(),
            direction_base.x(), direction_base.y(),
            camera_angle_deg, base_angle_deg);
        
        return true;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TF变换失败: %s", ex.what());
        return false;
    }
}


} // namespace robot_visioner

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<robot_visioner::RobotVisioner>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "💥 节点运行错误: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}