# Differential Drive Robot Simulation in ROS 2  

This package provides a ROS 2 Humble implementation for simulating a differential drive robot in Gazebo, featuring a velocity-based controller, waypoint navigation using PID control, and obstacle avoidance using LiDAR.  

## Overview  

The package includes:  
- **URDF/Xacro** model of a differential drive robot  
- **Gazebo integration** with necessary plugins  
- **C++ node** (`diff_drive_controller.cpp`) to convert `/cmd_vel` to wheel RPMs  
- **Python script** (`waypoint_navigation.py`) for waypoint-based navigation  
- **LiDAR sensor integration** for obstacle detection and avoidance  

## Running the Simulation  

1. **Launch Gazebo with the robot:**  

   ```bash
   ros2 launch mobile_dd_robot gazebo_mode.launch.py
   ```

2. **Run the waypoint navigation node:**  

   ```bash
   ros2 run mobile_dd_robot waypoint_navigation.py --ros-args \
   -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0  
   ```

## Nodes and Topics  

### `diff_drive_controller.cpp`  
- **Subscribes to:** `/cmd_vel` (geometry_msgs/Twist)  
- **Publishes:**  
  - `/left_wheel_rpm` (std_msgs/Float64)  
  - `/right_wheel_rpm` (std_msgs/Float64)  

### `waypoint_navigation.py`  
- **Subscribes to:**  
  - `/odom` (nav_msgs/Odometry)  
  - `/scan` (sensor_msgs/LaserScan) – for obstacle avoidance  

- **Publishes:** `/cmd_vel` (geometry_msgs/Twist)  

## Testing Commands  

- **Verify `/cmd_vel` to RPM conversion:**  

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'  
   ```

   Then check:  

   ```bash
   ros2 topic echo /left_wheel_rpm  
   ros2 topic echo /right_wheel_rpm  
   ```

- **Check LiDAR data:**  

   ```bash
   ros2 topic echo /scan  
   ```

- **Confirm Gazebo applies RPMs:**  

   ```bash
   ros2 topic list | grep rpm  
   ```

## License  

MIT License  
