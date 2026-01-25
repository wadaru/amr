#!/bin/bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/robot_3dw/link/body/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/robot_3dw/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args \
  -r /model/robot_3dw/link/body/cmd_vel:=/cmd_vel \
  -r /model/robot_3dw/odometry:=/odom & 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
