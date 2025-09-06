#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

class YOLODetectorNode(Node):
    """增强版YOLO检测节点"""
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 逐个声明参数 - 修复参数声明方法
        self.declare_parameter('model_path', 'yolov8n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('device', 'cpu')
        
        # 输入输出话题
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_detection_topic', '/yolo/detection_result')
        self.declare_parameter('output_mask_topic', '/yolo/mask')
        self.declare_parameter('output_combined_topic', '/yolo/combined_result')
        
        # 分割相关参数
        self.declare_parameter('enable_segmentation', True)
        self.declare_parameter('mask_threshold', 0.5)
        self.declare_parameter('target_classes', '')  # 空字符串表示所有类别
        self.declare_parameter('publish_individual_masks', False)
        
        # 图像参数
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # 性能参数
        self.declare_parameter('max_detections', 100)
        self.declare_parameter('agnostic_nms', False)
        
        # 获取参数值
        self.load_parameters()
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 尝试加载YOLO模型
        self.load_model()
        
        # 创建订阅者和发布者
        self.setup_topics()
        
        # 相机信息
        self.camera_info = None
        
        # 颜色映射用于可视化
        self.colors = self.generate_colors(80)
        
        self.log_initialization_info()
    
    def load_parameters(self):
        """获取参数值"""
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.device = self.get_parameter('device').value
        
        self.input_topic = self.get_parameter('input_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_detection_topic = self.get_parameter('output_detection_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value
        self.output_combined_topic = self.get_parameter('output_combined_topic').value
        
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.mask_threshold = self.get_parameter('mask_threshold').value
        self.target_classes_str = self.get_parameter('target_classes').value
        self.publish_individual_masks = self.get_parameter('publish_individual_masks').value
        
        # 解析target_classes字符串
        if self.target_classes_str and self.target_classes_str.strip():
            try:
                self.target_classes = [int(x.strip()) for x in self.target_classes_str.split(',')]
            except:
                self.target_classes = []
        else:
            self.target_classes = []
        
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.max_detections = self.get_parameter('max_detections').value
        self.agnostic_nms = self.get_parameter('agnostic_nms').value
    
    def load_model(self):
        """加载YOLO模型"""
        try:
            # 尝试导入ultralytics
            from ultralytics import YOLO
            
            self.get_logger().info(f'Loading YOLO model: {self.model_path}')
            self.model = YOLO(self.model_path)
            
            # 检查设备
            if self.device == 'cuda':
                try:
                    import torch
                    if torch.cuda.is_available():
                        self.model.to('cuda')
                        self.get_logger().info('Using CUDA device')
                    else:
                        self.get_logger().warn('CUDA not available, using CPU')
                        self.device = 'cpu'
                except:
                    self.get_logger().warn('PyTorch not available, using CPU')
                    self.device = 'cpu'
            
            self.get_logger().info(f'Model loaded successfully on device: {self.device}')
            self.model_loaded = True
            
        except ImportError:
            self.get_logger().error('ultralytics not installed. Using simple thresholding as fallback.')
            self.model_loaded = False
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}. Using fallback.')
            self.model_loaded = False
    
    def setup_topics(self):
        """设置订阅者和发布者"""
        # 订阅者
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)
            
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        
        # 发布者
        self.detection_result_pub = self.create_publisher(
            Image, self.output_detection_topic, 10)
            
        if self.enable_segmentation:
            self.mask_pub = self.create_publisher(
                Image, self.output_mask_topic, 10)
                
            self.combined_result_pub = self.create_publisher(
                Image, self.output_combined_topic, 10)
    
    def generate_colors(self, num_classes):
        """生成用于可视化的随机颜色"""
        colors = []
        np.random.seed(42)
        for _ in range(num_classes):
            color = tuple(np.random.randint(50, 255, 3).tolist())
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
            
            if self.model_loaded:
                # 使用YOLO模型
                results = self.model(
                    cv_image,
                    conf=self.confidence_threshold,
                    iou=self.iou_threshold,
                    max_det=self.max_detections,
                    agnostic_nms=self.agnostic_nms,
                    verbose=False
                )
                
                detection_image = self.process_detection_results(cv_image, results[0])
                
                if self.enable_segmentation:
                    combined_mask, combined_image = self.process_segmentation_results(cv_image, results[0])
                else:
                    combined_mask, combined_image = None, detection_image
            else:
                # 使用简单的阈值分割作为fallback
                detection_image, combined_mask, combined_image = self.simple_fallback_processing(cv_image)
            
            # 发布检测结果
            detection_msg = self.bridge.cv2_to_imgmsg(detection_image, "bgr8")
            detection_msg.header = msg.header
            self.detection_result_pub.publish(detection_msg)
            
            # 发布分割结果
            if self.enable_segmentation and combined_mask is not None:
                mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, "mono8")
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
                
                if combined_image is not None:
                    combined_msg = self.bridge.cv2_to_imgmsg(combined_image, "bgr8")
                    combined_msg.header = msg.header
                    self.combined_result_pub.publish(combined_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def simple_fallback_processing(self, image):
        """简单的fallback处理，当YOLO模型不可用时使用"""
        # 创建简单的阈值mask
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        
        # 在图像上绘制一些信息
        annotated = image.copy()
        cv2.putText(annotated, 'Fallback Mode (No YOLO)', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return annotated, mask, annotated
    
    def process_detection_results(self, image, results):
        """处理YOLO检测结果"""
        annotated_image = image.copy()
        
        if not hasattr(results, 'boxes') or results.boxes is None:
            return annotated_image
            
        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy().astype(int)
        
        # 过滤目标类别
        if self.target_classes:
            valid_indices = [i for i, cls in enumerate(classes) if cls in self.target_classes]
            boxes = boxes[valid_indices]
            scores = scores[valid_indices]
            classes = classes[valid_indices]
        
        # 绘制边界框
        for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
            x1, y1, x2, y2 = map(int, box)
            
            # 获取类别名称
            class_name = f'Class_{cls}'
            if hasattr(self.model, 'names'):
                class_name = self.model.names.get(cls, f'Class_{cls}')
            
            # 选择颜色
            color = self.colors[cls % len(self.colors)]
            
            # 绘制边界框
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制标签
            label = f'{class_name}: {score:.2f}'
            self.draw_label(annotated_image, label, (x1, y1), color)
        
        return annotated_image
    
    def process_segmentation_results(self, image, results):
        """处理分割结果"""
        height, width = image.shape[:2]
        combined_mask = np.zeros((height, width), dtype=np.uint8)
        annotated_image = image.copy()
        
        if not hasattr(results, 'masks') or results.masks is None:
            return None, annotated_image
        
        masks = results.masks.data.cpu().numpy()
        boxes = results.boxes.xyxy.cpu().numpy() if results.boxes is not None else None
        scores = results.boxes.conf.cpu().numpy() if results.boxes is not None else None
        classes = results.boxes.cls.cpu().numpy().astype(int) if results.boxes is not None else None
        
        if classes is None:
            return None, annotated_image
        
        valid_mask_count = 0
        
        for i, mask in enumerate(masks):
            if i >= len(classes):
                continue
                
            cls = classes[i]
            score = scores[i] if scores is not None else 1.0
            
            # 过滤目标类别
            if self.target_classes and cls not in self.target_classes:
                continue
            
            # 调整mask尺寸
            mask_resized = cv2.resize(mask, (width, height))
            mask_binary = (mask_resized > self.mask_threshold).astype(np.uint8) * 255
            
            # 合并到总mask中
            combined_mask = cv2.bitwise_or(combined_mask, mask_binary)
            valid_mask_count += 1
            
            # 在可视化图像上绘制mask
            color = self.colors[cls % len(self.colors)]
            colored_mask = np.zeros_like(image)
            colored_mask[mask_binary > 0] = color
            
            # 叠加mask
            annotated_image = cv2.addWeighted(annotated_image, 0.7, colored_mask, 0.3, 0)
        
        if valid_mask_count > 0:
            self.get_logger().info_throttle(2.0, f'Generated mask with {valid_mask_count} objects')
            return combined_mask, annotated_image
        else:
            return None, annotated_image
    
    def draw_label(self, image, label, position, color):
        """绘制标签文本"""
        x, y = position
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        
        # 获取文本尺寸
        (text_width, text_height), _ = cv2.getTextSize(label, font, font_scale, thickness)
        
        # 绘制标签背景
        cv2.rectangle(image, (x, y - text_height - 10), (x + text_width, y), color, -1)
        
        # 绘制标签文本
        cv2.putText(image, label, (x, y - 5), font, font_scale, (255, 255, 255), thickness)
    
    def log_initialization_info(self):
        """记录初始化信息"""
        self.get_logger().info('=== YOLO Detector Node ===')
        self.get_logger().info(f'Model: {self.model_path}')
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Model loaded: {self.model_loaded}')
        self.get_logger().info(f'Segmentation: {self.enable_segmentation}')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Mask output: {self.output_mask_topic}')

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