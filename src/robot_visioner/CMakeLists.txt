cmake_minimum_required(VERSION 3.8)
project(robot_visioner)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# PCL 定义和库
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# C++ 依赖项列表
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

# 创建点云处理节点
add_executable(point_cloud_processor src/point_cloud_processor_node.cpp)
ament_target_dependencies(point_cloud_processor ${dependencies})
target_link_libraries(point_cloud_processor 
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# 创建视觉处理主节点
add_executable(ai_visioner_node src/ai_visioner_node.cpp)
ament_target_dependencies(ai_visioner_node ${dependencies})
target_link_libraries(ai_visioner_node 
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# 安装C++可执行文件
install(TARGETS
  point_cloud_processor
  ai_visioner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  scripts/yolo_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装模型文件（如果有）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY
    models
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()