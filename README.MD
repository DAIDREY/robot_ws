```bash
conda deactivate
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_description display.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_task robot_task.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_visioner robot_visioner.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_driver robot_driver.launch.py
```

# 手眼标定
```bash
conda deactivate
source install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py \
    name:=seed_robot \
    calibration_type:=eye_on_base \
    tracking_base_frame:=camera_color_optical_frame \
    tracking_marker_frame:=aruco_marker_frame \
    robot_base_frame:=base_link \
    robot_effector_frame:=aruco_marker_frame
```

```bash
conda deactivate
source install/setup.bash
ros2 launch aruco_ros single.launch.py
```

# 转轴测试指令
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
points:
- positions: [0.0, 50.99, 110.85, -9.22, 59.06, 20.0]
  time_from_start: {sec: 3, nanosec: 0}
" --once
```

# 目标位置测试指令
```bash
ros2 topic pub /pose_target geometry_msgs/msg/PoseStamped "
pose:
  position: {x: 0.526, y: -0.147, z: 0.124}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
" --once
```

# 抓取测试指令
```bash
ros2 service call /grasp_object robot_task/srv/GraspObject "{object_name: ''}"
```