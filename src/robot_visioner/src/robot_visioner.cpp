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
    
    // 订阅者
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, 10, 
        std::bind(&RobotVisioner::depthCallback, this, std::placeholders::_1));
        
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        mask_topic, 10,
        std::bind(&RobotVisioner::maskCallback, this, std::placeholders::_1));
        
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&RobotVisioner::cameraInfoCallback, this, std::placeholders::_1));
    
    // 发布者
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, 10);
        
    center_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/robot_visioner/center_point", 10);
        
    center_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/robot_visioner/center_marker", 10);
        
    cluster_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/robot_visioner/cluster_markers", 10);
    
    last_process_time_ = this->now();
    
    logNodeInfo();
}

void RobotVisioner::initializeParameters()
{
    // 声明和获取参数
    this->declare_parameter("depth_topic", "/camera/depth/image_raw");
    this->declare_parameter("mask_topic", "/yolo/mask");
    this->declare_parameter("camera_info_topic", "/camera/camera_info");
    this->declare_parameter("output_topic", "/extracted_pointcloud");
    
    // 处理参数
    this->declare_parameter("depth_scale", 1000.0);
    this->declare_parameter("mask_threshold", 128);
    this->declare_parameter("sync_tolerance", 0.05);
    
    // 聚类参数
    this->declare_parameter("enable_clustering", true);
    this->declare_parameter("cluster_tolerance", 0.02);
    this->declare_parameter("min_cluster_size", 100);
    this->declare_parameter("max_cluster_size", 25000);
    
    // 过滤参数
    this->declare_parameter("enable_voxel_filter", true);
    this->declare_parameter("voxel_leaf_size", 0.01);
    this->declare_parameter("enable_outlier_filter", true);
    this->declare_parameter("outlier_mean_k", 50);
    this->declare_parameter("outlier_stddev_mul", 1.0);
    
    // 工作空间参数
    this->declare_parameter("workspace_x_min", -2.0);
    this->declare_parameter("workspace_x_max", 2.0);
    this->declare_parameter("workspace_y_min", -2.0);
    this->declare_parameter("workspace_y_max", 2.0);
    this->declare_parameter("workspace_z_min", 0.1);
    this->declare_parameter("workspace_z_max", 3.0);
    
    // 可视化参数
    this->declare_parameter("marker_scale", 0.05);
    this->declare_parameter("frame_id", "camera_link");
    
    // 获取参数值
    depth_scale_ = this->get_parameter("depth_scale").as_double();
    mask_threshold_ = this->get_parameter("mask_threshold").as_int();
    sync_tolerance_ = this->get_parameter("sync_tolerance").as_double();
    
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
}

void RobotVisioner::logNodeInfo()
{
    RCLCPP_INFO(this->get_logger(), "=== Robot Visioner 节点已启动 ===");
    RCLCPP_INFO(this->get_logger(), "深度缩放: %.1f", depth_scale_);
    RCLCPP_INFO(this->get_logger(), "Mask阈值: %d", mask_threshold_);
    RCLCPP_INFO(this->get_logger(), "聚类分析: %s", enable_clustering_ ? "启用" : "禁用");
    RCLCPP_INFO(this->get_logger(), "体素过滤: %s", enable_voxel_filter_ ? "启用" : "禁用");
    RCLCPP_INFO(this->get_logger(), "坐标系: %s", frame_id_.c_str());
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
    RCLCPP_INFO_ONCE(this->get_logger(), "相机内参已更新: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                     fx_, fy_, cx_, cy_);
}

void RobotVisioner::tryExtractPointCloud()
{
    // 检查是否所有数据都已准备好
    if (!depth_image_ || !mask_image_ || !camera_info_received_) {
        return;
    }
    
    // 检查时间戳同步
    auto depth_time = rclcpp::Time(depth_image_->header.stamp);
    auto mask_time = rclcpp::Time(mask_image_->header.stamp);
    
    if (std::abs((depth_time - mask_time).seconds()) > sync_tolerance_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "深度图和mask时间戳不同步: %.3f秒差异", 
                             std::abs((depth_time - mask_time).seconds()));
        return;
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
        
        // 确保尺寸匹配
        if (depth_img.size() != mask_img.size()) {
            cv::resize(mask_img, mask_img, depth_img.size());
        }
        
        // 创建点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 遍历所有像素生成点云
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
                
                // 添加到点云
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterPointCloud(cloud);
        
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
        
        // 发布中心点
        if (!center_points.empty()) {
            publishCenterPoints(center_points, output_msg.header);
            publishCenterMarkers(center_points, output_msg.header);
        }
        
        // 统计信息
        processed_frames_++;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "处理完成: %zu个点 | %zu个中心点 | 耗时: %ldms | 总帧数: %zu",
                            filtered_cloud->points.size(), center_points.size(), 
                            duration.count(), processed_frames_);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "点云提取失败: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RobotVisioner::filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = input_cloud;
    
    // 体素滤波
    if (enable_voxel_filter_ && !input_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*voxel_filtered);
        filtered_cloud = voxel_filtered;
    }
    
    // 离群点过滤
    if (enable_outlier_filter_ && filtered_cloud->size() > outlier_mean_k_) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(filtered_cloud);
        outlier_filter.setMeanK(outlier_mean_k_);
        outlier_filter.setStddevMulThresh(outlier_stddev_mul_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        outlier_filter.filter(*outlier_filtered);
        filtered_cloud = outlier_filtered;
    }
    
    return filtered_cloud;
}

CenterPoint3D RobotVisioner::calculateCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
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
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<CenterPoint3D> centers;
    
    if (cloud->empty()) {
        return centers;
    }
    
    // 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // 计算每个聚类的中心点
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
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
    
    // 发布主要中心点（置信度最高的）
    auto best_center = *std::max_element(centers.begin(), centers.end(),
        [](const CenterPoint3D& a, const CenterPoint3D& b) {
            return a.confidence < b.confidence;
        });
    
    geometry_msgs::msg::PointStamped center_msg;
    center_msg.header = header;
    center_msg.point.x = best_center.x;
    center_msg.point.y = best_center.y;
    center_msg.point.z = best_center.z;
    
    center_point_pub_->publish(center_msg);
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
        
        marker.scale.x = marker_scale_ * 2;
        marker.scale.y = marker_scale_ * 2;
        marker.scale.z = marker_scale_ * 2;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
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
            
            marker.scale.x = marker_scale_;
            marker.scale.y = marker_scale_;
            marker.scale.z = marker_scale_;
            
            // 根据置信度设置颜色
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = std::min(1.0, center.confidence * 2.0);
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            marker_array.markers.push_back(marker);
        }
        
        cluster_markers_pub_->publish(marker_array);
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
        RCLCPP_ERROR(node->get_logger(), "节点运行错误: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}