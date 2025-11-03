#!/bin/bash
python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('test_path_pub')
pub = node.create_publisher(Path, '/plan', 10)

path = Path()
path.header.frame_id = 'map'
path.header.stamp = node.get_clock().now().to_msg()

# Define a simple test path with several waypoints
for x, y in [(1.5, 2.0), (0.5, 2.0), (0.0, 0.0), (1.0, 1.0), (0.0, 1.0)]:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = path.header.stamp
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = 1.0
    path.poses.append(ps)

pub.publish(path)
print(f'Published path with {len(path.poses)} poses')

rclpy.shutdown()
EOF
