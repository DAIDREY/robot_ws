cmake_minimum_required(VERSION 3.8)
project(motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanCartesianPath.srv"
  "srv/PlanJointPath.srv"
  "srv/PlanGraspSequence.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs trajectory_msgs
)

# 包含目录
include_directories(include)

# 可执行文件
add_executable(motion_planner_node
  src/main.cpp
  src/motion_planner.cpp
  # src/trajectory_optimizer.cpp
)

# 依赖项
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  moveit_core
)

ament_target_dependencies(motion_planner_node ${dependencies})

# 链接生成的接口
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motion_planner_node "${cpp_typesupport_target}")

# 安装
install(TARGETS motion_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(${dependencies})
ament_package()