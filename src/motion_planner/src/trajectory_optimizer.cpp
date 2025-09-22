#include "trajectory_optimizer.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace motion_planner
{

TrajectoryOptimizer::TrajectoryOptimizer(rclcpp::Logger logger)
    : logger_(logger)
{
}

bool TrajectoryOptimizer::optimizeTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory, 
                                           int max_points)
{
    if (trajectory.points.empty()) {
        RCLCPP_WARN(logger_, "轨迹为空，无法优化");
        return false;
    }
    
    if (static_cast<int>(trajectory.points.size()) <= max_points) {
        RCLCPP_DEBUG(logger_, "轨迹点数(%zu)已在限制内(%d)，无需优化", 
                    trajectory.points.size(), max_points);
        return true;
    }
    
    RCLCPP_INFO(logger_, "优化轨迹: %zu -> %d 点", trajectory.points.size(), max_points);
    
    // 使用重采样方法优化
    return resampleTrajectory(trajectory, max_points);
}

bool TrajectoryOptimizer::resampleTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                                           int target_points)
{
    if (trajectory.points.empty() || target_points <= 0) {
        return false;
    }
    
    if (static_cast<int>(trajectory.points.size()) <= target_points) {
        return true;
    }
    
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> original_points = trajectory.points;
    trajectory.points.clear();
    trajectory.points.reserve(target_points);
    
    // 总是保留第一个和最后一个点
    trajectory.points.push_back(original_points.front());
    
    if (target_points > 2) {
        // 计算采样间隔
        double step = static_cast<double>(original_points.size() - 1) / (target_points - 1);
        
        // 添加中间点
        for (int i = 1; i < target_points - 1; ++i) {
            double index = i * step;
            int lower_index = static_cast<int>(std::floor(index));
            int upper_index = std::min(lower_index + 1, static_cast<int>(original_points.size() - 1));
            double ratio = index - lower_index;
            
            trajectory_msgs::msg::JointTrajectoryPoint interpolated_point;
            if (interpolatePoint(original_points[lower_index], original_points[upper_index], 
                               ratio, interpolated_point)) {
                trajectory.points.push_back(interpolated_point);
            } else {
                trajectory.points.push_back(original_points[lower_index]);
            }
        }
    }
    
    // 添加最后一个点
    if (target_points > 1) {
        trajectory.points.push_back(original_points.back());
    }
    
    // 重新计算时间戳
    updateTimeStamps(trajectory);
    
    RCLCPP_DEBUG(logger_, "轨迹重采样完成: %zu 点", trajectory.points.size());
    return true;
}

bool TrajectoryOptimizer::smoothTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                                         double smoothing_factor)
{
    if (trajectory.points.size() < 3) {
        return true;  // 点数太少，无需平滑
    }
    
    RCLCPP_DEBUG(logger_, "平滑轨迹，平滑因子: %.2f", smoothing_factor);
    
    auto original_points = trajectory.points;
    
    // 对每个关节进行平滑
    for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
        for (size_t j = 0; j < trajectory.points[i].positions.size(); ++j) {
            double prev = original_points[i-1].positions[j];
            double curr = original_points[i].positions[j];
            double next = original_points[i+1].positions[j];
            
            // 简单的3点平滑滤波
            trajectory.points[i].positions[j] = 
                (1.0 - smoothing_factor) * curr + 
                smoothing_factor * 0.5 * (prev + next);
        }
    }
    
    return true;
}

bool TrajectoryOptimizer::validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty()) {
        RCLCPP_ERROR(logger_, "轨迹验证失败: 轨迹为空");
        return false;
    }
    
    if (trajectory.joint_names.empty()) {
        RCLCPP_ERROR(logger_, "轨迹验证失败: 关节名称为空");
        return false;
    }
    
    size_t expected_joints = trajectory.joint_names.size();
    
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        const auto& point = trajectory.points[i];
        
        if (point.positions.size() != expected_joints) {
            RCLCPP_ERROR(logger_, "轨迹验证失败: 点%zu关节数量不匹配", i);
            return false;
        }
        
        // 检查时间戳单调性
        if (i > 0) {
            auto prev_time = rclcpp::Duration(trajectory.points[i-1].time_from_start);
            auto curr_time = rclcpp::Duration(point.time_from_start);
            
            if (curr_time <= prev_time) {
                RCLCPP_ERROR(logger_, "轨迹验证失败: 时间戳非单调递增");
                return false;
            }
        }
    }
    
    RCLCPP_DEBUG(logger_, "轨迹验证通过");
    return true;
}

bool TrajectoryOptimizer::checkVelocityLimits(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                             const std::vector<double>& max_velocities)
{
    if (max_velocities.empty()) {
        // 使用默认限制
        std::vector<double> default_limits(trajectory.joint_names.size(), DEFAULT_MAX_VELOCITY);
        return checkVelocityLimits(trajectory, default_limits);
    }
    
    for (const auto& point : trajectory.points) {
        for (size_t j = 0; j < point.velocities.size() && j < max_velocities.size(); ++j) {
            if (std::abs(point.velocities[j]) > max_velocities[j]) {
                RCLCPP_WARN(logger_, "关节%zu速度超限: %.3f > %.3f", 
                           j, std::abs(point.velocities[j]), max_velocities[j]);
                return false;
            }
        }
    }
    
    return true;
}

bool TrajectoryOptimizer::checkAccelerationLimits(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                 const std::vector<double>& max_accelerations)
{
    if (max_accelerations.empty()) {
        // 使用默认限制
        std::vector<double> default_limits(trajectory.joint_names.size(), DEFAULT_MAX_ACCELERATION);
        return checkAccelerationLimits(trajectory, default_limits);
    }
    
    for (const auto& point : trajectory.points) {
        for (size_t j = 0; j < point.accelerations.size() && j < max_accelerations.size(); ++j) {
            if (std::abs(point.accelerations[j]) > max_accelerations[j]) {
                RCLCPP_WARN(logger_, "关节%zu加速度超限: %.3f > %.3f", 
                           j, std::abs(point.accelerations[j]), max_accelerations[j]);
                return false;
            }
        }
    }
    
    return true;
}

double TrajectoryOptimizer::calculateTrajectoryLength(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.size() < 2) {
        return 0.0;
    }
    
    double total_length = 0.0;
    
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double point_distance = calculatePointDistance(trajectory.points[i-1], trajectory.points[i]);
        total_length += point_distance;
    }
    
    return total_length;
}

double TrajectoryOptimizer::calculateExecutionTime(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty()) {
        return 0.0;
    }
    
    auto last_time = rclcpp::Duration(trajectory.points.back().time_from_start);
    return last_time.seconds();
}

std::vector<double> TrajectoryOptimizer::calculateJointDisplacements(
    const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.size() < 2) {
        return std::vector<double>(trajectory.joint_names.size(), 0.0);
    }
    
    std::vector<double> displacements(trajectory.joint_names.size(), 0.0);
    
    const auto& start_point = trajectory.points.front();
    const auto& end_point = trajectory.points.back();
    
    for (size_t j = 0; j < displacements.size() && j < start_point.positions.size(); ++j) {
        displacements[j] = std::abs(end_point.positions[j] - start_point.positions[j]);
    }
    
    return displacements;
}

bool TrajectoryOptimizer::reparameterizeTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                                                  double velocity_scaling_factor,
                                                  double acceleration_scaling_factor)
{
    if (trajectory.points.size() < 2) {
        return true;
    }
    
    RCLCPP_DEBUG(logger_, "重新参数化轨迹，速度缩放: %.2f，加速度缩放: %.2f", 
                velocity_scaling_factor, acceleration_scaling_factor);
    
    // 简单的时间缩放方法
    double time_scaling = 1.0 / std::max(velocity_scaling_factor, 0.1);
    
    for (auto& point : trajectory.points) {
        // 缩放时间
        auto old_time = rclcpp::Duration(point.time_from_start);
        auto new_time = rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(old_time.nanoseconds() * time_scaling));
        point.time_from_start = new_time;
        
        // 缩放速度
        for (auto& vel : point.velocities) {
            vel *= velocity_scaling_factor;
        }
        
        // 缩放加速度
        for (auto& acc : point.accelerations) {
            acc *= acceleration_scaling_factor;
        }
    }
    
    return true;
}

double TrajectoryOptimizer::calculatePointDistance(const trajectory_msgs::msg::JointTrajectoryPoint& p1,
                                                  const trajectory_msgs::msg::JointTrajectoryPoint& p2)
{
    if (p1.positions.size() != p2.positions.size()) {
        return 0.0;
    }
    
    double distance = 0.0;
    for (size_t i = 0; i < p1.positions.size(); ++i) {
        double diff = p2.positions[i] - p1.positions[i];
        distance += diff * diff;
    }
    
    return std::sqrt(distance);
}

bool TrajectoryOptimizer::interpolatePoint(const trajectory_msgs::msg::JointTrajectoryPoint& p1,
                                         const trajectory_msgs::msg::JointTrajectoryPoint& p2,
                                         double ratio,
                                         trajectory_msgs::msg::JointTrajectoryPoint& result)
{
    if (p1.positions.size() != p2.positions.size()) {
        return false;
    }
    
    result.positions.resize(p1.positions.size());
    
    // 线性插值位置
    for (size_t i = 0; i < p1.positions.size(); ++i) {
        result.positions[i] = p1.positions[i] + ratio * (p2.positions[i] - p1.positions[i]);
    }
    
    // 插值时间
    auto time1 = rclcpp::Duration(p1.time_from_start);
    auto time2 = rclcpp::Duration(p2.time_from_start);
    auto time_diff = time2 - time1;
    auto interpolated_time = time1 + rclcpp::Duration::from_nanoseconds(
        static_cast<int64_t>(time_diff.nanoseconds() * ratio));
    result.time_from_start = interpolated_time;
    
    // 插值速度（如果有的话）
    if (!p1.velocities.empty() && !p2.velocities.empty() && 
        p1.velocities.size() == p2.velocities.size()) {
        result.velocities.resize(p1.velocities.size());
        for (size_t i = 0; i < p1.velocities.size(); ++i) {
            result.velocities[i] = p1.velocities[i] + ratio * (p2.velocities[i] - p1.velocities[i]);
        }
    }
    
    // 插值加速度（如果有的话）
    if (!p1.accelerations.empty() && !p2.accelerations.empty() && 
        p1.accelerations.size() == p2.accelerations.size()) {
        result.accelerations.resize(p1.accelerations.size());
        for (size_t i = 0; i < p1.accelerations.size(); ++i) {
            result.accelerations[i] = p1.accelerations[i] + ratio * (p2.accelerations[i] - p1.accelerations[i]);
        }
    }
    
    return true;
}

void TrajectoryOptimizer::updateTimeStamps(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty()) {
        return;
    }
    
    // 计算总路径长度
    double total_length = calculateTrajectoryLength(trajectory);
    if (total_length == 0.0) {
        // 如果无法计算路径长度，使用均匀时间分布
        for (size_t i = 0; i < trajectory.points.size(); ++i) {
            double time_sec = static_cast<double>(i) * MIN_TIME_STEP;
            trajectory.points[i].time_from_start = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(time_sec * 1e9));
        }
        return;
    }
    
    // 基于路径长度重新分配时间
    double cumulative_length = 0.0;
    trajectory.points[0].time_from_start = rclcpp::Duration::from_nanoseconds(0);
    
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double segment_length = calculatePointDistance(trajectory.points[i-1], trajectory.points[i]);
        cumulative_length += segment_length;
        
        // 假设恒定速度
        double progress = cumulative_length / total_length;
        double time_sec = progress * std::max(total_length / DEFAULT_MAX_VELOCITY, 
                                             static_cast<double>(trajectory.points.size()) * MIN_TIME_STEP);
        
        trajectory.points[i].time_from_start = rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(time_sec * 1e9));
    }
}

void TrajectoryOptimizer::computeVelocities(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.size() < 2) {
        return;
    }
    
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        auto& point = trajectory.points[i];
        point.velocities.resize(point.positions.size(), 0.0);
        
        if (i == 0) {
            // 前向差分
            auto dt = rclcpp::Duration(trajectory.points[1].time_from_start - 
                                     trajectory.points[0].time_from_start).seconds();
            if (dt > 0) {
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    point.velocities[j] = (trajectory.points[1].positions[j] - point.positions[j]) / dt;
                }
            }
        } else if (i == trajectory.points.size() - 1) {
            // 后向差分
            auto dt = rclcpp::Duration(trajectory.points[i].time_from_start - 
                                     trajectory.points[i-1].time_from_start).seconds();
            if (dt > 0) {
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    point.velocities[j] = (point.positions[j] - trajectory.points[i-1].positions[j]) / dt;
                }
            }
        } else {
            // 中心差分
            auto dt = rclcpp::Duration(trajectory.points[i+1].time_from_start - 
                                     trajectory.points[i-1].time_from_start).seconds();
            if (dt > 0) {
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    point.velocities[j] = (trajectory.points[i+1].positions[j] - 
                                          trajectory.points[i-1].positions[j]) / dt;
                }
            }
        }
    }
}

void TrajectoryOptimizer::computeAccelerations(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.size() < 2) {
        return;
    }
    
    // 首先确保速度已计算
    if (trajectory.points[0].velocities.empty()) {
        computeVelocities(trajectory);
    }
    
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
        auto& point = trajectory.points[i];
        point.accelerations.resize(point.positions.size(), 0.0);
        
        if (i == 0) {
            // 前向差分
            if (trajectory.points.size() > 1) {
                auto dt = rclcpp::Duration(trajectory.points[1].time_from_start - 
                                         trajectory.points[0].time_from_start).seconds();
                if (dt > 0) {
                    for (size_t j = 0; j < point.velocities.size(); ++j) {
                        point.accelerations[j] = (trajectory.points[1].velocities[j] - 
                                                 point.velocities[j]) / dt;
                    }
                }
            }
        } else if (i == trajectory.points.size() - 1) {
            // 后向差分
            auto dt = rclcpp::Duration(trajectory.points[i].time_from_start - 
                                     trajectory.points[i-1].time_from_start).seconds();
            if (dt > 0) {
                for (size_t j = 0; j < point.velocities.size(); ++j) {
                    point.accelerations[j] = (point.velocities[j] - 
                                             trajectory.points[i-1].velocities[j]) / dt;
                }
            }
        } else {
            // 中心差分
            auto dt = rclcpp::Duration(trajectory.points[i+1].time_from_start - 
                                     trajectory.points[i-1].time_from_start).seconds();
            if (dt > 0) {
                for (size_t j = 0; j < point.velocities.size(); ++j) {
                    point.accelerations[j] = (trajectory.points[i+1].velocities[j] - 
                                             trajectory.points[i-1].velocities[j]) / dt;
                }
            }
        }
    }
}

} // namespace motion_planner