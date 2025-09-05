#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import yaml

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 声明参数
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/yolov8/image')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('device', 'cuda')  # 'cpu' or 'cuda'
        
        # 获取参数值
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        try:
            if os.path.isabs(self.model_path):
                model_full_path = self.model_path
            else:
                # 相对路径，假设在包的根目录下
                model_full_path = os.path.join(
                    get_package_share_directory('robot_visioner'), 
                    self.model_path
                )
            
            self.get_logger().info(f'Loading YOLO model from: {model_full_path}')
            self.model = YOLO(model_full_path)
            self.model.to(self.device)
            self.get_logger().info(f'Model loaded successfully on device: {self.device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            raise e
        
        # 创建订阅者和发布者
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        self.result_pub = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        # 相机信息
        self.camera_info = None
        
        # 颜色映射用于可视化
        self.colors = self.generate_colors(80)  # COCO数据集有80个类别
        
        self.get_logger().info('YOLO Detector Node initialized')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Publishing to: {self.output_topic}')
    
    def generate_colors(self, num_classes):
        """生成用于可视化的随机颜色"""
        colors = []
        np.random.seed(42)  # 固定随机种子以获得一致的颜色
        for _ in range(num_classes):
            color = tuple(np.random.randint(0, 255, 3).tolist())
            colors.append(color)
        return colors
    
    def camera_info_callback(self, msg):
        """相机信息回调函数"""
        self.camera_info = msg
    
    def image_callback(self, msg):
        """图像消息回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 运行YOLO推理
            results = self.model(
                cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            
            # 处理检测结果
            annotated_image = self.process_results(cv_image, results[0])
            
            # 将结果转换回ROS图像消息并发布
            result_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            result_msg.header = msg.header
            self.result_pub.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_results(self, image, results):
        """处理YOLO检测结果并绘制在图像上"""
        annotated_image = image.copy()
        
        if results.boxes is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            scores = results.boxes.conf.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy().astype(int)
            
            # 绘制边界框和标签
            for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
                x1, y1, x2, y2 = map(int, box)
                
                # 获取类别名称
                class_name = self.model.names[cls] if cls < len(self.model.names) else f'Class_{cls}'
                
                # 选择颜色
                color = self.colors[cls % len(self.colors)]
                
                # 绘制边界框
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                
                # 创建标签文本
                label = f'{class_name}: {score:.2f}'
                
                # 获取文本尺寸
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2
                (text_width, text_height), _ = cv2.getTextSize(label, font, font_scale, thickness)
                
                # 绘制标签背景
                cv2.rectangle(
                    annotated_image,
                    (x1, y1 - text_height - 10),
                    (x1 + text_width, y1),
                    color,
                    -1
                )
                
                # 绘制标签文本
                cv2.putText(
                    annotated_image,
                    label,
                    (x1, y1 - 5),
                    font,
                    font_scale,
                    (255, 255, 255),
                    thickness
                )
        
        # 处理分割结果（如果有）
        if hasattr(results, 'masks') and results.masks is not None:
            masks = results.masks.data.cpu().numpy()
            
            for i, mask in enumerate(masks):
                if i < len(classes):
                    cls = classes[i]
                    color = self.colors[cls % len(self.colors)]
                    
                    # 调整mask尺寸以匹配原图像
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    
                    # 创建彩色mask
                    colored_mask = np.zeros_like(image)
                    colored_mask[mask_resized > 0.5] = color
                    
                    # 叠加mask到图像上
                    annotated_image = cv2.addWeighted(annotated_image, 0.8, colored_mask, 0.2, 0)
        
        return annotated_image

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()