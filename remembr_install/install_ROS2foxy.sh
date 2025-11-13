#!/bin/bash
set -e

echo "=========================================="
echo "Installing ROS2 Foxy alongside ROS1 Noetic"
echo "=========================================="

# Add ROS2 repository
echo "Adding ROS2 repository..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS2 Foxy
echo "Updating package lists..."
apt-get update

echo "Installing ROS2 Foxy (this may take a few minutes)..."
apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-rosbag2 \
    ros-foxy-rosbag2-storage-default-plugins \
    python3-colcon-common-extensions

# Clean up
rm -rf /var/lib/apt/lists/*

# Create helper scripts
echo "Creating environment switching scripts..."

cat > /app/use_remembr << 'EOF'
#!/bin/bash
source /opt/ros/foxy/setup.bash
source /opt/venv/remembr/bin/activate
echo "Switched to Remembr environment (ROS2 Foxy + Python 3.10)"
EOF

chmod +x /app/use_remembr

# Add aliases to bashrc if not already present
if ! grep -q "alias remembr=" /root/.bashrc; then
    echo "alias remembr='source /app/use_remembr'" >> /root/.bashrc
fi

echo ""
echo "=========================================="
echo "Installation complete!"
echo "=========================================="
echo ""
echo "To use Remembr with ROS2, run:"
echo "  source /app/use_remembr"
echo "or simply:"
echo "  remembr"
echo ""
echo "Then navigate to the demo and run:"
echo "  cd /app/remembr/examples/chat_demo"
echo "  python3 demo.py --rosbag_enabled"
echo ""