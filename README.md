# Differential Drive Robot Simulation in ROS 2

This package provides a ROS 2 Humble implementation for simulating a differential drive robot in Gazebo, featuring a velocity-based controller, waypoint navigation using PID control, and obstacle avoidance using LiDAR.

## Overview

The package includes:
- **URDF/Xacro Model:** Differential drive robot model.
- **Gazebo Integration:** SDF/Gazebo plugin configuration to control the robot and publish odometry.
- **C++ Node:** `diff_drive_controller.cpp` which subscribes to `/cmd_vel`, converts it into wheel RPMs, and publishes:
  - `/left_wheel_rpm` (std_msgs/Float64)
  - `/right_wheel_rpm` (std_msgs/Float64)
- **Custom Gazebo Plugin:** `rpm_diff_drive_plugin.cpp` applies RPM values to the robot's wheel joints and publishes odometry on `/odom`.
- **Python Script:** `waypoint_navigation.py` for waypoint-based navigation using PID control and LiDAR-based obstacle avoidance. It subscribes to:
  - `/odom` (nav_msgs/Odometry)
  - `/scan` (sensor_msgs/LaserScan) – for obstacle detection and avoidance
  and publishes `/cmd_vel` (geometry_msgs/Twist).

## Building the Package

In your workspace root (where the package is located), run:

```bash
colcon build --packages-select mobile_dd_robot
```

After the build completes, source your workspace:

```bash
source install/setup.bash
```

## Running the Simulation

1. **Launch Gazebo with the Robot:**

   ```bash
   ros2 launch mobile_dd_robot gazebo_mode.launch.py
   ```

2. **Run the Differential Drive Controller Node:**

   In a new terminal (make sure to source your workspace):

   ```bash
   ros2 run mobile_dd_robot diff_drive_controller
   ```

3. **Run the Waypoint Navigation Node:**

   In another terminal (with the workspace sourced):

   ```bash
   ros2 run mobile_dd_robot waypoint_navigation.py --ros-args -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0
   ```

## Nodes and Topics

### `diff_drive_controller.cpp`
- **Subscribes to:** `/cmd_vel` (geometry_msgs/Twist)
- **Publishes:**
  - `/left_wheel_rpm` (std_msgs/Float64)
  - `/right_wheel_rpm` (std_msgs/Float64)

### `rpm_diff_drive_plugin.cpp`
- **Applies RPM values** to the wheel joints.
- **Publishes:** `/odom` (nav_msgs/Odometry) using differential drive kinematics.
- **Note:** This plugin is loaded via the SDF (robot.gazebo) file.

### `waypoint_navigation.py`
- **Subscribes to:**
  - `/odom` (nav_msgs/Odometry)
  - `/scan` (sensor_msgs/LaserScan) – for obstacle avoidance
- **Publishes:** `/cmd_vel` (geometry_msgs/Twist)
- **Behavior:** Uses PID control to navigate between waypoints and adjusts commands for obstacle avoidance based on LiDAR data.

## Testing Commands

- **Verify `/cmd_vel` to RPM Conversion:**

  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
  ros2 topic echo /left_wheel_rpm
  ros2 topic echo /right_wheel_rpm
  ```

- **Check Odometry Data:**

  ```bash
  ros2 topic echo /odom
  ```

- **Check LiDAR Data:**

  ```bash
  ros2 topic echo /scan
  ```

- **List RPM Topics (to confirm plugin is applied):**

  ```bash
  ros2 topic list | grep rpm
  ```

## Additional Notes

- **Obstacle Avoidance:**  
  The waypoint navigation node processes LiDAR data to detect obstacles and adjust the command velocities. If the robot is manually controlled via `/cmd_vel` (with `ros2 topic pub`), it bypasses the waypoint navigation logic. Ensure that your simulation is publishing odometry correctly (via the custom plugin) so that the waypoint navigator can compute commands based on the robot’s pose.

- **Gazebo Plugin Configuration:**  
  The `robot.gazebo` file should load your custom RPM plugin:

  ```xml
  <gazebo>
    <plugin name="rpm_diff_drive" filename="librpm_diff_drive_plugin.so">
      <left_joint>wheel2_joint</left_joint>
      <right_joint>wheel1_joint</right_joint>
      <wheel_radius>0.1</wheel_radius>
      <wheelbase>0.5</wheelbase>
    </plugin>
  </gazebo>
  ```

  This ensures that RPM values computed by the `diff_drive_controller` node are applied to the wheels, and odometry is published.

## License

MIT License
