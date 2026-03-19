#!/bin/bash

# Configure TCP for both arms

# Set TCP on arm_1
echo "=== Setting TCP on arm_1 ==="
ros2 service call arm_1/niryo_robot_tools_commander/set_tcp niryo_ned_ros2_interfaces/srv/SetTCP \
  '{position: {x: 0.0726, y: -0.0007261, z: -0.01155}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, tcp_version: 2}'
echo ""

# Enable TCP on arm_1
echo "=== Enabling TCP on arm_1 ==="
ros2 service call arm_1/niryo_robot_tools_commander/enable_tcp niryo_ned_ros2_interfaces/srv/SetBool \
  '{value: true}'
echo ""

# Set TCP on arm_2
echo "=== Setting TCP on arm_2 ==="
ros2 service call arm_2/niryo_robot_tools_commander/set_tcp niryo_ned_ros2_interfaces/srv/SetTCP \
  '{position: {x: 0.0726, y: -0.0007261, z: -0.01155}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, tcp_version: 2}'
echo ""

# Enable TCP on arm_2
echo "=== Enabling TCP on arm_2 ==="
ros2 service call arm_2/niryo_robot_tools_commander/enable_tcp niryo_ned_ros2_interfaces/srv/SetBool \
  '{value: true}'
echo ""

echo "All service calls completed!"
