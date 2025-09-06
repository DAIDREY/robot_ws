cmake_minimum_required(VERSION 3.8)
project(robot_visioner)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
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
  visualization_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

# 创建主要的robot_visioner节点
add_executable(robot_visioner src/robot_visioner.cpp)
ament_target_dependencies(robot_visioner ${dependencies})
target_link_libraries(robot_visioner 
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES}
)

# 安装C++可执行文件
install(TARGETS
  robot_visioner
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/yolo_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detector
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

# 安装rviz配置文件
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()