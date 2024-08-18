#!/bin/bash

# Check if argument is provided
if [ $# -eq 0 ]; then
    echo "Please provide an argument between 1 and 6."
    exit 1
fi

# Check if argument is a number between 1 and 6
if ! [[ $1 =~ ^[1-6]$ ]]; then
    echo "Invalid argument. Please provide a number between 1 and 6."
    exit 1
fi

# Set ROS_DOMAIN_ID environment variable
export ROS_DOMAIN_ID=$1

# Publish to /cmd_vel topic with value 0
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


