```bash
conda deactivate
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py   color_width:=1280   color_height:=720   color_info_url:=file:///home/ubuntu/robot_ws/src/OrbbecSDK_ROS2/orbbec_camera/config/camera_calibration.yaml
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_visioner robot_visioner.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_task robot_task.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_description display.launch.py
```

```bash
conda deactivate
source install/setup.bash
ros2 launch robot_driver robot_driver.launch.py
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