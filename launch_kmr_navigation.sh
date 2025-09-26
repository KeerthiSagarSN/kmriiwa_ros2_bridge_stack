#!/bin/bash

# ROS2 KMR IIWA Launcher Script
# This script launches all necessary ROS2 nodes in separate terminals

echo "🚀 Starting ROS2 KMR IIWA Application..."
echo "================================================"

# Function to open new terminal and run command
open_terminal_and_run() {
    local cmd="$1"
    local title="$2"
    
    # For GNOME Terminal
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$cmd; echo 'Press Enter to close...'; read"
    # For other terminals (xterm, konsole, etc.)
    elif command -v xterm &> /dev/null; then
        xterm -title "$title" -e bash -c "$cmd; echo 'Press Enter to close...'; read" &
    else
        echo "❌ No supported terminal found. Please install gnome-terminal or xterm"
        exit 1
    fi
}

# Step 1: Start static transform publishers
echo "📡 Starting static transform publishers..."
TF_CMD="ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id kmriiwa_odom & ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id kmriiwa_odom --child-frame-id kmriiwa_base_footprint"

open_terminal_and_run "$TF_CMD" "TF Publishers"

# Wait a moment for TF publishers to start
sleep 2

# Step 2: Launch robot navigation
echo "🤖 Launching robot navigation..."
NAV_CMD="ros2 launch kmriiwa_navigation kmr_robot_navigation.launch.py"
open_terminal_and_run "$NAV_CMD" "Robot Navigation"

# Wait a moment for navigation to initialize
sleep 3

# Step 3: Launch laserscan merge
echo "📊 Launching laserscan merge..."
LASER_CMD="ros2 launch kmriiwa_navigation laserscan_merge.launch.py"
open_terminal_and_run "$LASER_CMD" "Laserscan Merge"

echo ""
echo "✅ All ROS2 nodes launched successfully!"
echo "💡 To stop all processes, close the terminal windows or use Ctrl+C in each terminal"
echo ""
echo "🔧 Terminals opened:"
echo "   1. TF Publishers"
echo "   2. Robot Navigation" 
echo "   3. Laserscan Merge"
echo ""
echo "📋 Original commands:"
echo "   Terminal 1: $TF_CMD"
echo "   Terminal 2: $NAV_CMD"
echo "   Terminal 3: $LASER_CMD"
