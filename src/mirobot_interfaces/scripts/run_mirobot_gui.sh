#!/bin/bash

# Get the source directory
SOURCE_DIR="/home/jonas/Desktop/Dev/Mirobot/src/mirobot_interfaces"

# Activate the virtual environment
source "$SOURCE_DIR/.venv/bin/activate"

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash
source /home/jonas/Desktop/Dev/Mirobot/install/setup.bash

# Run the GUI script
python3 "$SOURCE_DIR/scripts/mirobot_gui.py" "$@"
