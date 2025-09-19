#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math
import json
from scipy.spatial.transform import Rotation

class YOLODetectorNode(Node):
    """增强版YOLO检测节点 - 支持物体名称识别和旋转角度"""
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 初始化日志时间戳
        self._last_log_time = 0
        self._last_info_time = 0
        
        # 逐个声明参数
        self.declare_parameter('model_path', 'yolov8n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('device', 'cpu')
        
        # 输入输出话题
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('output_detection_topic', '/yolo/detection_result')
        self.declare_parameter('output_mask_topic', '/yolo/mask')
        self.declare_parameter('output_combined_topic', '/yolo/combined_result')
        
        # 分割相关参数
        self.declare_parameter('enable_segmentation', True)
        self.declare_parameter('mask_threshold', 0.5)
        self.declare_parameter('target_classes', '')
        self.declare_parameter('publish_individual_masks', False)
        
        # 图像参数
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # 性能参数
        self.declare_parameter('max_detections', 100)
        self.declare_parameter('agnostic_nms', False)
        
        # RViz标记参数
        self.declare_parameter('output_marker_topic', '/yolo/markers')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('enable_rviz_markers', True)
        self.declare_parameter('marker_scale', 0.1)
        self.declare_parameter('show_rotation_text', True)
        self.declare_parameter('default_object_depth', 1.0)
        
        # 获取参数值
        self.load_parameters()
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # YOLO类别ID到物体名称的映射
        self.class_id_to_object_name = {
            0: "luosi",
            1: "pen",
            2: "xiguan",
            3: "zhoucheng",
        }
        
        # 目标物体过滤（只处理这些物体用于抓取任务）
        self.target_objects = ["luosi", "pen", "xiguan", "zhoucheng"]
        
        # 尝试加载YOLO模型
        self.load_model()
        
        # 创建订阅者和发布者
        self.setup_topics()
        
        # 相机信息
        self.camera_info = None
        
        # 颜色映射用于可视化
        self.colors = self.generate_colors(80)
        
        self.log_initialization_info()
    
    def throttled_log_info(self, message, throttle_sec=2.0):
        """节流日志信息"""
        current_time = time.time()
        if current_time - self._last_info_time > throttle_sec:
            self.get_logger().info(message)
            self._last_info_time = current_time
    
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
        self.publish_individual_masks = self.get_parameter('publish_individual_masks').value
        
        # 解析目标类别
        target_classes_str = self.get_parameter('target_classes').value
        if target_classes_str:
            try:
                self.target_classes = [int(x.strip()) for x in target_classes_str.split(',')]
            except:
                self.target_classes = []
        else:
            self.target_classes = []
        
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.max_detections = self.get_parameter('max_detections').value
        self.agnostic_nms = self.get_parameter('agnostic_nms').value
        
        # RViz参数
        self.output_marker_topic = self.get_parameter('output_marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.enable_rviz_markers = self.get_parameter('enable_rviz_markers').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.show_rotation_text = self.get_parameter('show_rotation_text').value
        self.default_object_depth = self.get_parameter('default_object_depth').value
    
    def load_model(self):
        """加载YOLO模型"""
        try:
            from ultralytics import YOLO
            
            self.get_logger().info(f'🤖 Loading YOLO model: {self.model_path}')
            self.model = YOLO(self.model_path)
            
            # 检查设备
            if self.device == 'cuda':
                try:
                    import torch
                    if torch.cuda.is_available():
                        self.model.to('cuda')
                        self.get_logger().info('✅ Using CUDA device')
                    else:
                        self.get_logger().warn('⚠️ CUDA not available, using CPU')
                        self.device = 'cpu'
                except:
                    self.get_logger().warn('⚠️ PyTorch not available, using CPU')
                    self.device = 'cpu'
            
            self.get_logger().info(f'✅ Model loaded successfully on device: {self.device}')
            self.get_logger().info(f'📋 Model classes: {self.model.names}')
            self.model_loaded = True
            
        except ImportError:
            self.get_logger().error('❌ ultralytics not installed. Using simple fallback.')
            self.model_loaded = False
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load YOLO model: {str(e)}. Using fallback.')
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
        
        # RViz标记发布者
        if self.enable_rviz_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, self.output_marker_topic, 10)
        
        # 新增：检测信息发布者（给robot_visioner）
        self.detection_info_pub = self.create_publisher(
            String, '/yolo/detection_info', 10)
        
        self.get_logger().info(f'📡 Subscribed to: {self.input_topic}')
        self.get_logger().info(f'📤 Publishing detection to: {self.output_detection_topic}')
        if self.enable_segmentation:
            self.get_logger().info(f'📤 Publishing mask to: {self.output_mask_topic}')
        if self.enable_rviz_markers:
            self.get_logger().info(f'📤 Publishing markers to: {self.output_marker_topic}')
        self.get_logger().info(f'📤 Publishing detection info to: /yolo/detection_info')
    
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
    
    def calculate_rotation_angle(self, mask):
        """计算物体的旋转角度"""
        try:
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return 0.0, None
            
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 使用最小外接矩形计算旋转角度
            rect = cv2.minAreaRect(largest_contour)
            center, (width, height), angle = rect
            
            # OpenCV的角度范围是[-90, 0]，转换为[0, 180]
            if width < height:
                angle = angle + 90
            
            # 标准化角度到[0, 360]范围
            angle = angle % 360
            
            return angle, center
            
        except Exception as e:
            self.get_logger().debug(f'角度计算失败: {str(e)}')
            return 0.0, None
    
    def pixel_to_world(self, pixel_x, pixel_y, depth=None):
        """将像素坐标转换为世界坐标"""
        if self.camera_info is None:
            return None
        
        if depth is None:
            depth = self.default_object_depth
        
        # 从相机内参矩阵获取焦距和主点
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        # 转换为相机坐标系
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth
        
        return Point(x=x, y=y, z=z)
    
    def publish_detection_info(self, detections_data, timestamp):
        """发布检测信息给robot_visioner"""
        if not detections_data:
            return
        
        # 找到置信度最高的目标物体
        best_detection = None
        best_score = 0
        
        for bbox, class_id, confidence, angle, center in detections_data:
            object_name = self.class_id_to_object_name.get(class_id, "unknown")
            
            # 只处理目标物体
            if object_name in self.target_objects and confidence > best_score:
                best_detection = {
                    'object_name': object_name,
                    'center_x': float(center[0]),
                    'center_y': float(center[1]), 
                    'rotation_angle': float(angle),
                    'confidence': float(confidence),
                    'class_id': int(class_id),
                    'bbox': [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])]
                }
                best_score = confidence
        
        if best_detection:
            # 发布检测信息（JSON格式）
            detection_msg = String()
            detection_msg.data = json.dumps(best_detection)
            self.detection_info_pub.publish(detection_msg)
            
            self.get_logger().debug(f"发布检测信息: {best_detection['object_name']} "
                                  f"角度: {best_detection['rotation_angle']:.1f}°")
    
    def process_detection_results(self, image, results, detections_for_markers):
        """处理检测结果并绘制"""
        annotated_image = image.copy()
        
        if not hasattr(results, 'boxes') or results.boxes is None:
            return annotated_image
        
        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy().astype(int)
        
        # 检查数组是否为空
        if len(boxes) == 0 or len(scores) == 0 or len(classes) == 0:
            self.get_logger().debug('⚠️ 未发现检测结果，返回原始图像')
            return annotated_image
        
        self.get_logger().debug(f'🎯 原始检测: {len(boxes)} 个边界框')
        
        # 安全地计算min/max，避免空数组错误
        if len(scores) > 0:
            self.get_logger().debug(f'得分范围: {scores.min():.3f}-{scores.max():.3f}')
        
        # 过滤目标类别时添加检查
        if self.target_classes:
            valid_indices = [i for i, cls in enumerate(classes) if cls in self.target_classes]
            if len(valid_indices) == 0:
                self.throttled_log_info('⚠️ 没有物体匹配目标类别')
                return annotated_image
                
            boxes = boxes[valid_indices]
            scores = scores[valid_indices]
            classes = classes[valid_indices]
            self.throttled_log_info(f'🎯 After class filtering: {len(boxes)} boxes')
        
        # 获取分割mask用于角度计算
        masks = getattr(results, 'masks', None)
        
        # 绘制边界框
        for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
            x1, y1, x2, y2 = map(int, box)
            
            # 获取物体名称
            object_name = self.class_id_to_object_name.get(cls, f'Class_{cls}')
            
            # 选择颜色
            color = self.colors[cls % len(self.colors)]
            
            # 计算旋转角度
            angle = 0.0
            center = ((x1 + x2) / 2, (y1 + y2) / 2)
            
            if masks is not None and i < len(masks.data):
                # 使用分割掩码计算角度
                mask = masks.data[i].cpu().numpy()
                mask_resized = cv2.resize(
                    mask, (image.shape[1], image.shape[0]))
                mask_binary = (mask_resized > self.mask_threshold).astype(np.uint8) * 255
                
                angle, mask_center = self.calculate_rotation_angle(mask_binary)
                if mask_center:
                    center = mask_center
            else:
                # 使用边界框估计角度
                roi = image[int(y1):int(y2), int(x1):int(x2)]
                if roi.size > 0:
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    _, binary_roi = cv2.threshold(gray_roi, 0, 255, 
                                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    
                    angle, roi_center = self.calculate_rotation_angle(binary_roi)
                    if roi_center:
                        center = (x1 + roi_center[0], y1 + roi_center[1])
            
            # 存储检测结果用于RViz标记和信息发布
            bbox = [x1, y1, x2, y2]
            detections_for_markers.append((bbox, cls, score, angle, center))
            
            # 绘制边界框
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制旋转角度指示线
            center_int = (int(center[0]), int(center[1]))
            angle_rad = math.radians(angle)
            line_length = 30
            end_x = int(center[0] + line_length * math.cos(angle_rad))
            end_y = int(center[1] + line_length * math.sin(angle_rad))
            cv2.arrowedLine(annotated_image, center_int, (end_x, end_y), 
                          (0, 0, 255), 3, tipLength=0.3)
            
            # 绘制标签 - 显示物体名称
            label = f'{object_name}: {score:.2f} {angle:.1f}°'
            self.draw_label(annotated_image, label, (x1, y1), color)
        
        return annotated_image
    
    def process_segmentation_results(self, image, results):
        """处理分割结果"""
        height, width = image.shape[:2]
        combined_mask = np.zeros((height, width), dtype=np.uint8)
        annotated_image = image.copy()
        
        if not hasattr(results, 'masks') or results.masks is None:
            self.throttled_log_info('⚠️ No segmentation masks found')
            return None, annotated_image
        
        masks = results.masks.data.cpu().numpy()
        if len(masks) == 0:
            self.throttled_log_info('⚠️ 掩码数组为空')
            return None, annotated_image
        boxes = results.boxes.xyxy.cpu().numpy() if results.boxes is not None else None
        scores = results.boxes.conf.cpu().numpy() if results.boxes is not None else None
        classes = results.boxes.cls.cpu().numpy().astype(int) if results.boxes is not None else None
        
        if classes is None:
            self.throttled_log_info('⚠️ No class information for masks')
            return None, annotated_image
        
        self.get_logger().debug(f'Processing {len(masks)} masks')
        
        valid_mask_count = 0
        
        for i, mask in enumerate(masks):
            if i >= len(classes):
                continue
                
            cls = classes[i]
            score = scores[i] if scores is not None else 1.0
            
            # 过滤目标类别
            if self.target_classes and cls not in self.target_classes:
                continue
            
            # 获取物体名称
            object_name = self.class_id_to_object_name.get(cls, f'Class_{cls}')
            
            # 调整mask尺寸
            mask_resized = cv2.resize(mask, (width, height))
            mask_binary = (mask_resized > self.mask_threshold).astype(np.uint8) * 255
            
            # 检查mask是否有效
            mask_pixels = np.sum(mask_binary > 0)
            if mask_pixels == 0:
                continue
            
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
            total_mask_pixels = np.sum(combined_mask > 0)
            self.get_logger().debug(f'✅ Generated final mask: {valid_mask_count} objects, {total_mask_pixels} pixels')
            return combined_mask, annotated_image
        else:
            self.throttled_log_info('⚠️ No valid masks generated')
            return None, annotated_image
    
    def create_detection_markers(self, detections, timestamp):
        """创建RViz标记"""
        marker_array = MarkerArray()
        
        for i, detection in enumerate(detections):
            bbox, class_id, confidence, angle, center = detection
            
            # 获取物体名称
            object_name = self.class_id_to_object_name.get(class_id, f'class_{class_id}')
            
            # 创建边界框标记
            bbox_marker = Marker()
            bbox_marker.header.frame_id = self.frame_id
            bbox_marker.header.stamp = timestamp
            bbox_marker.ns = "detection_boxes"
            bbox_marker.id = i * 2
            bbox_marker.type = Marker.CUBE
            bbox_marker.action = Marker.ADD
            
            # 计算3D位置
            center_3d = self.pixel_to_world(center[0], center[1])
            if center_3d:
                bbox_marker.pose.position = center_3d
            else:
                bbox_marker.pose.position = Point(x=0.0, y=0.0, z=1.0)
            
            # 设置旋转（绕Z轴旋转）
            angle_rad = math.radians(angle)
            r = Rotation.from_euler('z', angle_rad)
            quat = r.as_quat()  # [x, y, z, w]
            bbox_marker.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            
            # 设置尺寸
            bbox_width = (bbox[2] - bbox[0]) * self.marker_scale * 0.001
            bbox_height = (bbox[3] - bbox[1]) * self.marker_scale * 0.001
            bbox_marker.scale.x = max(bbox_width, 0.05)
            bbox_marker.scale.y = max(bbox_height, 0.05)
            bbox_marker.scale.z = 0.02
            
            # 设置颜色
            color = self.colors[class_id % len(self.colors)]
            bbox_marker.color = ColorRGBA(
                r=color[2]/255.0, g=color[1]/255.0, b=color[0]/255.0, a=0.7)
            
            bbox_marker.lifetime.sec = 1
            marker_array.markers.append(bbox_marker)
            
            # 创建文本标记
            if self.show_rotation_text:
                text_marker = Marker()
                text_marker.header.frame_id = self.frame_id
                text_marker.header.stamp = timestamp
                text_marker.ns = "detection_text"
                text_marker.id = i * 2 + 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                if center_3d:
                    text_marker.pose.position = Point(
                        x=center_3d.x, y=center_3d.y - 0.1, z=center_3d.z + 0.05)
                else:
                    text_marker.pose.position = Point(x=0.0, y=-0.1, z=1.05)
                
                text_marker.scale.z = 0.05
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                
                text_marker.text = f'{object_name}\n{confidence:.2f}\n{angle:.1f}°'
                text_marker.lifetime.sec = 1
                
                marker_array.markers.append(text_marker)
        
        return marker_array
    
    def fallback_detection(self, image):
        """简单的fallback检测"""
        detection_image = image.copy()
        
        # 在图像上绘制"No Model"文本
        cv2.putText(detection_image, "YOLO Model Not Available", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return detection_image, None, detection_image
    
    def image_callback(self, msg):
        """图像消息回调函数"""
        try:
            # 验证输入图像
            if msg.width == 0 or msg.height == 0:
                self.get_logger().error('🚫 接收到尺寸为零的空图像')
                return
                
            self.get_logger().debug(f' 接收图像: {msg.width}x{msg.height}')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 验证转换后的图像
            if cv_image.size == 0:
                self.get_logger().error('🚫 转换后的图像为空')
                return
            
            # 存储检测结果用于RViz标记
            detections_for_markers = []
            
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
                
                detection_image = self.process_detection_results(cv_image, results[0], detections_for_markers)
                
                # 发布检测信息给robot_visioner
                if detections_for_markers:
                    self.publish_detection_info(detections_for_markers, msg.header.stamp)
                
                if self.enable_segmentation:
                    combined_mask, combined_image = self.process_segmentation_results(cv_image, results[0])
                else:
                    combined_mask, combined_image = None, detection_image
            else:
                # 使用简单的fallback处理
                detection_image, combined_mask, combined_image = self.fallback_detection(cv_image)
            
            # 发布结果
            self.publish_results(detection_image, combined_mask, combined_image, msg.header.stamp)
            
            # 发布RViz标记
            if self.enable_rviz_markers and detections_for_markers:
                marker_array = self.create_detection_markers(detections_for_markers, msg.header.stamp)
                self.marker_pub.publish(marker_array)
                
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
            import traceback
            self.get_logger().error(f'堆栈跟踪: {traceback.format_exc()}')
    
    def publish_results(self, detection_image, combined_mask, combined_image, timestamp):
        """发布处理结果"""
        try:
            # 发布检测结果图像
            detection_msg = self.bridge.cv2_to_imgmsg(detection_image, "bgr8")
            detection_msg.header.stamp = timestamp
            self.detection_result_pub.publish(detection_msg)
            
            # 发布分割结果
            if self.enable_segmentation and combined_mask is not None:
                mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, "mono8")
                mask_msg.header.stamp = timestamp
                self.mask_pub.publish(mask_msg)
                
                if combined_image is not None:
                    combined_msg = self.bridge.cv2_to_imgmsg(combined_image, "bgr8")
                    combined_msg.header.stamp = timestamp
                    self.combined_result_pub.publish(combined_msg)
                    
        except Exception as e:
            self.get_logger().error(f'发布结果失败: {str(e)}')
    
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
        self.get_logger().info('=== YOLO Detector Node with Object Names ===')
        self.get_logger().info(f'📦 Model: {self.model_path}')
        self.get_logger().info(f'🔧 Device: {self.device}')
        self.get_logger().info(f'✅ Model loaded: {self.model_loaded}')
        self.get_logger().info(f'🎭 Segmentation: {self.enable_segmentation}')
        self.get_logger().info(f'🎯 Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'📋 Target classes: {self.target_classes if self.target_classes else "All classes"}')
        self.get_logger().info(f'🤖 Target objects: {self.target_objects}')
        self.get_logger().info(f'🔄 RViz markers: {self.enable_rviz_markers}')

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