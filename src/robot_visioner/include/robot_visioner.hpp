// include/robot_visioner/robot_visioner.hpp
#ifndef ROBOT_VISIONER_HPP
#define ROBOT_VISIONER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_task/msg/object_pose.hpp>
#include <mutex>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robot_visioner
{

struct CenterPoint3D
{
    double x, y, z;
    size_t point_count;
    double confidence;
    
    CenterPoint3D(double x = 0.0, double y = 0.0, double z = 0.0, 
                  size_t count = 0, double conf = 0.0)
        : x(x), y(y), z(z), point_count(count), confidence(conf) {}
};

class RobotVisioner : public rclcpp::Node
{
public:
    RobotVisioner();
    virtual ~RobotVisioner() = default;

private:
    struct DetectionInfo {
        std::string object_name;
        double center_x, center_y;
        double rotation_angle;
        double confidence;
        bool valid = false;
        rclcpp::Time timestamp; 
    };
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string target_frame_;  // 目标坐标系 (base_link)
    std::string source_frame_;  // 源坐标系 (camera_depth_optical_frame)

    DetectionInfo latest_detection_;
    std::mutex detection_mutex_;
    // 回调函数
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void detectionInfoCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // 主要处理函数
    void tryExtractPointCloud();
    void extractMaskedPointCloud();
    
    // 中心点计算与可视化
    CenterPoint3D calculateCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    std::vector<CenterPoint3D> extractClusterCentroids(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void publishCenterPoints(const std::vector<CenterPoint3D>& centers, 
                           const std_msgs::msg::Header& header);
    void publishCenterMarkers(const std::vector<CenterPoint3D>& centers, 
                            const std_msgs::msg::Header& header);
    
    // 点云处理辅助函数
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
    void validateCenterPoint(CenterPoint3D& center);
    cv::Vec3b getDepthColor(double depth, double min_depth, double max_depth);
    
    bool parseDetectionInfo(const std::string& json_str, DetectionInfo& info);

    bool transformPointToBaseLink(const geometry_msgs::msg::PointStamped& point_in,
                                  geometry_msgs::msg::PointStamped& point_out);
                                  
    bool transformRotationToBaseLink(double camera_rotation_angle, 
                                    const std::string& camera_frame,
                                    double& base_link_rotation_angle);

    // 参数初始化
    void initializeParameters();
    void logNodeInfo();

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_info_sub_;
    
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr center_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_markers_pub_;
    rclcpp::Publisher<robot_task::msg::ObjectPose>::SharedPtr object_pose_pub_;
    
    
    // 数据缓存
    sensor_msgs::msg::Image::SharedPtr rgb_image_;
    sensor_msgs::msg::Image::SharedPtr depth_image_;
    sensor_msgs::msg::Image::SharedPtr mask_image_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    
    // 相机参数
    double fx_, fy_, cx_, cy_;
    bool camera_info_received_;
    
    // 可配置参数
    std::string rgb_topic_;            // RGB图像话题
    double depth_scale_;               // 深度缩放因子
    int mask_threshold_;               // Mask二值化阈值
    double sync_tolerance_;            // 时间同步容差
    bool enable_rgb_color_;            // 是否启用RGB着色
    
    // 点云处理参数
    bool enable_clustering_;           // 是否启用聚类
    double cluster_tolerance_;         // 聚类容差
    int min_cluster_size_;             // 最小聚类大小
    int max_cluster_size_;             // 最大聚类大小
    
    // 过滤参数
    bool enable_voxel_filter_;         // 体素滤波
    double voxel_leaf_size_;           // 体素大小
    bool enable_outlier_filter_;       // 离群点过滤
    int outlier_mean_k_;               // 离群点检测邻域点数
    double outlier_stddev_mul_;        // 离群点标准差倍数
    
    // 工作空间限制
    double workspace_x_min_, workspace_x_max_;
    double workspace_y_min_, workspace_y_max_;
    double workspace_z_min_, workspace_z_max_;
    
    // 可视化参数
    double marker_scale_;              // 标记大小
    std::string frame_id_;             // 坐标系ID
    
    // 统计信息
    size_t processed_frames_;
    rclcpp::Time last_process_time_;
    
    double position_offset_x_;    // X方向偏移量 (米)
    double position_offset_y_;    // Y方向偏移量 (米) 
    double position_offset_z_;    // Z方向偏移量 (米)
    bool enable_position_offset_;
};

} // namespace robot_visioner

#endif // ROBOT_VISIONER_HPP