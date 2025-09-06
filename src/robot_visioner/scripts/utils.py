#!/usr/bin/env python3

import numpy as np
import cv2
from typing import List, Tuple, Optional

class ColorPalette:
    """颜色调色板类，用于生成和管理检测框的颜色"""
    
    def __init__(self, n_colors: int = 80):
        self.n_colors = n_colors
        self.colors = self._generate_colors()
    
    def _generate_colors(self) -> List[Tuple[int, int, int]]:
        """生成颜色列表"""
        colors = []
        np.random.seed(42)  # 固定种子确保颜色一致
        
        for _ in range(self.n_colors):
            color = tuple(np.random.randint(0, 255, 3).tolist())
            colors.append(color)
        
        return colors
    
    def get_color(self, class_id: int) -> Tuple[int, int, int]:
        """根据类别ID获取颜色"""
        return self.colors[class_id % len(self.colors)]

class ImageProcessor:
    """图像处理工具类"""
    
    @staticmethod
    def resize_image(image: np.ndarray, target_size: Tuple[int, int], 
                    keep_aspect_ratio: bool = True) -> np.ndarray:
        """
        调整图像尺寸
        
        Args:
            image: 输入图像
            target_size: 目标尺寸 (width, height)
            keep_aspect_ratio: 是否保持宽高比
        
        Returns:
            调整后的图像
        """
        if keep_aspect_ratio:
            h, w = image.shape[:2]
            target_w, target_h = target_size
            
            # 计算缩放比例
            scale = min(target_w / w, target_h / h)
            
            # 计算新的尺寸
            new_w = int(w * scale)
            new_h = int(h * scale)
            
            # 调整图像尺寸
            resized = cv2.resize(image, (new_w, new_h))
            
            # 创建目标尺寸的图像，用黑色填充
            result = np.zeros((target_h, target_w, 3), dtype=image.dtype)
            
            # 计算居中位置
            y_offset = (target_h - new_h) // 2
            x_offset = (target_w - new_w) // 2
            
            # 将调整后的图像放置在中心
            result[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized
            
            return result
        else:
            return cv2.resize(image, target_size)
    
    @staticmethod
    def draw_detection_box(image: np.ndarray, box: List[float], 
                          class_name: str, confidence: float, 
                          color: Tuple[int, int, int], 
                          thickness: int = 2) -> np.ndarray:
        """
        在图像上绘制检测框
        
        Args:
            image: 输入图像
            box: 边界框坐标 [x1, y1, x2, y2]
            class_name: 类别名称
            confidence: 置信度
            color: 颜色 (B, G, R)
            thickness: 线条粗细
        
        Returns:
            绘制后的图像
        """
        x1, y1, x2, y2 = map(int, box)
        
        # 绘制边界框
        cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
        
        # 准备标签文本
        label = f'{class_name}: {confidence:.2f}'
        
        # 获取文本尺寸
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        text_thickness = 1
        (text_width, text_height), _ = cv2.getTextSize(
            label, font, font_scale, text_thickness
        )
        
        # 绘制标签背景
        cv2.rectangle(
            image,
            (x1, y1 - text_height - 10),
            (x1 + text_width, y1),
            color,
            -1
        )
        
        # 绘制标签文本
        cv2.putText(
            image,
            label,
            (x1, y1 - 5),
            font,
            font_scale,
            (255, 255, 255),
            text_thickness
        )
        
        return image
    
    @staticmethod
    def apply_mask(image: np.ndarray, mask: np.ndarray, 
                  color: Tuple[int, int, int], alpha: float = 0.3) -> np.ndarray:
        """
        在图像上应用分割掩码
        
        Args:
            image: 输入图像
            mask: 分割掩码
            color: 掩码颜色 (B, G, R)
            alpha: 透明度
        
        Returns:
            应用掩码后的图像
        """
        # 确保mask尺寸匹配
        if mask.shape[:2] != image.shape[:2]:
            mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
        
        # 创建彩色掩码
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0.5] = color
        
        # 叠加掩码
        result = cv2.addWeighted(image, 1 - alpha, colored_mask, alpha, 0)
        
        return result

class ROSImageConverter:
    """ROS图像转换工具"""
    
    @staticmethod
    def ros_to_cv2(ros_image, encoding: str = "bgr8"):
        """将ROS图像消息转换为OpenCV格式"""
        from cv_bridge import CvBridge
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(ros_image, encoding)
    
    @staticmethod
    def cv2_to_ros(cv_image, encoding: str = "bgr8", header=None):
        """将OpenCV图像转换为ROS消息格式"""
        from cv_bridge import CvBridge
        from std_msgs.msg import Header
        
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding)
        
        if header is not None:
            ros_image.header = header
        else:
            ros_image.header = Header()
            ros_image.header.stamp = rclpy.clock.Clock().now().to_msg()
        
        return ros_image

def filter_detections_by_confidence(boxes, scores, classes, 
                                  threshold: float = 0.5) -> Tuple[np.ndarray, ...]:
    """
    根据置信度过滤检测结果
    
    Args:
        boxes: 边界框数组
        scores: 置信度数组
        classes: 类别数组
        threshold: 置信度阈值
    
    Returns:
        过滤后的 (boxes, scores, classes)
    """
    mask = scores >= threshold
    return boxes[mask], scores[mask], classes[mask]

def filter_detections_by_area(boxes, min_area: float = 100.0) -> np.ndarray:
    """
    根据面积过滤检测框
    
    Args:
        boxes: 边界框数组 [[x1, y1, x2, y2], ...]
        min_area: 最小面积阈值
    
    Returns:
        有效检测框的布尔掩码
    """
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    return areas >= min_area