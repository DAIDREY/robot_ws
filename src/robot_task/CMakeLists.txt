cmake_minimum_required(VERSION 3.8)
project(robot_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPose.msg"
  "msg/TaskStatus.msg"
  "srv/GraspObject.srv"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# 包含头文件目录
include_directories(include)

# 依赖项列表
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# 创建可执行文件
add_executable(task_manager 
  src/task_manager.cpp
  src/main.cpp
)
ament_target_dependencies(task_manager ${dependencies})

# 链接生成的消息
rosidl_target_interfaces(task_manager ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装可执行文件
install(TARGETS task_manager
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include/
  DESTINATION include)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()