#!/bin/bash
# Script to install Gazebo Classic on Ubuntu 22.04
# Since gazebo11 is not available for Jammy, we'll try installing from Focal (20.04) packages

set -e

echo "=== Checking Ubuntu Version ==="
lsb_release -a

echo ""
echo "=== Step 1: Add Gazebo Repository ==="
# Add Gazebo repository for Focal (where gazebo11 exists)
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    echo "Adding Gazebo repository..."
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget -q https://packages.osrfoundation.org/gazebo.key -O /tmp/gazebo.key
    sudo mv /tmp/gazebo.key /etc/apt/trusted.gpg.d/gazebo.gpg
fi

echo ""
echo "=== Step 2: Update package list ==="
sudo apt-get update

echo ""
echo "=== Step 3: Install Gazebo Classic (gazebo11) from Focal ==="
echo "Attempting to install gazebo11 from Ubuntu 20.04 (Focal) repository..."
echo "This may work if dependencies are compatible with Ubuntu 22.04"
sudo apt-get install -y gazebo11 libgazebo11-dev || {
    echo ""
    echo "=== Installation failed - trying with dependency resolution fixes ==="
    echo "Attempting to fix broken dependencies..."
    sudo apt-get install -f -y
    sudo apt-get install -y gazebo11 libgazebo11-dev --fix-broken || {
        echo ""
        echo "=== Installation failed. Gazebo Classic requires Ubuntu 20.04 or building from source ==="
        echo ""
        echo "Alternative options:"
        echo "1. Use Docker with Ubuntu 20.04"
        echo "2. Build from source (see instructions below)"
        echo "3. Use Gazebo Harmonics (already installed)"
        exit 1
    }
}

echo ""
echo "=== Step 4: Install ROS2 integration ==="
sudo apt-get install -y ros-humble-gazebo-ros-pkgs || echo "Warning: ros-humble-gazebo-ros-pkgs installation failed"

echo ""
echo "=== Verification ==="
if which gazebo > /dev/null 2>&1; then
    echo "✓ Gazebo Classic installed successfully!"
    gazebo --version
    ros2 pkg list | grep gazebo_ros && echo "✓ ROS2 integration found" || echo "✗ ROS2 integration not found"
else
    echo "✗ Gazebo Classic installation failed"
    exit 1
fi

