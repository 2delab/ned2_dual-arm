#!/bin/bash

# Calibrate motors on both arms in parallel

ros2 service call arm_1/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}" &
ros2 service call arm_2/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}" &
