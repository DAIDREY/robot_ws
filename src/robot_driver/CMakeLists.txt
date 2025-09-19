cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)                    
find_package(tf2_geometry_msgs REQUIRED)
# 尝试查找libserial (可选)
find_library(SERIAL_LIBRARY 
    NAMES serial
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)

find_path(SERIAL_INCLUDE_DIR 
    NAMES libserial/SerialPort.h
    PATHS /usr/include /usr/local/include
)

# 设置编译选项
if(SERIAL_LIBRARY AND SERIAL_INCLUDE_DIR)
    message(STATUS "找到libserial: ${SERIAL_LIBRARY}")
    message(STATUS "libserial头文件: ${SERIAL_INCLUDE_DIR}")
    set(USE_REAL_SERIAL ON)
else()
    message(STATUS "未找到libserial，使用模拟实现")
    set(USE_REAL_SERIAL OFF)
    add_definitions(-DUSE_MOCK_SERIAL)
endif()

# 包含目录
include_directories(include)
if(USE_REAL_SERIAL)
    include_directories(${SERIAL_INCLUDE_DIR})
endif()

# 创建库
add_library(${PROJECT_NAME}_lib
    src/serial_communication.cpp
    src/seed_robot_driver.cpp
)

# 设置库的依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2                    
  tf2_geometry_msgs      
)

# 链接libserial (如果找到)
if(USE_REAL_SERIAL AND SERIAL_LIBRARY)
    target_link_libraries(${PROJECT_NAME}_lib ${SERIAL_LIBRARY})
endif()

# 创建可执行文件
add_executable(seed_robot_driver_node src/main.cpp)
target_link_libraries(seed_robot_driver_node ${PROJECT_NAME}_lib)

ament_target_dependencies(seed_robot_driver_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

# 安装目标
install(TARGETS
  ${PROJECT_NAME}_lib
  seed_robot_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装启动文件和配置文件
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

ament_package()