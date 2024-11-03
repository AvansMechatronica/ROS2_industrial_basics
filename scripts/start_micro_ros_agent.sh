#!/bin/bash

# Default values
DEFAULT_DEVICE="/dev/ttyACM0"
DEFAULT_VERBOSITY="v6"

# Check if the first argument (device) is provided; if not, use the default
DEVICE_PATH="${1:-$DEFAULT_DEVICE}"

# Check if the second argument (verbosity) is provided; if not, use the default
VERBOSITY_LEVEL="${2:-$DEFAULT_VERBOSITY}"

# Inform the user of the device and verbosity level being used
echo "Using device: $DEVICE_PATH"
echo "Using verbosity level: $VERBOSITY_LEVEL"

# Run the micro-ROS agent command with specified or default arguments
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE_PATH" -"$VERBOSITY_LEVEL"
