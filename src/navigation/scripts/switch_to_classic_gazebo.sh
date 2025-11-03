#!/bin/bash
# Script to uninstall Gazebo Harmonics and install classic Gazebo

echo "=== Uninstalling Gazebo Harmonics ==="
# Uninstall all Gazebo Harmonics packages
sudo apt-get remove -y gz-harmonic gz-sim8* gz-launch7* gz-tools* gz-transport* gz-gui* gz-msgs* gz-physics* gz-rendering* gz-plugin* gz-math* gz-cmake* gz-fuel-tools* 2>/dev/null

echo ""
echo "=== Adding Gazebo Repository ==="
# Add Gazebo repository for Ubuntu 22.04 (Jammy)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# Use modern method for adding GPG key
wget https://packages.osrfoundation.org/gazebo.key -O /tmp/gazebo.key
sudo mv /tmp/gazebo.key /etc/apt/trusted.gpg.d/gazebo.gpg

echo ""
echo "=== Installing Classic Gazebo ==="
# Update and install classic Gazebo and ROS2 integration
sudo apt-get update
sudo apt-get install -y gazebo11 libgazebo11-dev ros-humble-gazebo-ros-pkgs

echo ""
echo "=== Verification ==="
echo "Checking installations..."
which gazebo && echo "✓ Classic Gazebo installed" || echo "✗ Classic Gazebo not found"
ros2 pkg list | grep gazebo_ros && echo "✓ gazebo_ros packages found" || echo "✗ gazebo_ros packages not found"

echo ""
echo "Done! You can now use the classic Gazebo launch file."

