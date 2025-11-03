#!/usr/bin/env python3
import sys
import time
import subprocess
import os
import re
from ament_index_python.packages import get_package_share_directory

def main():
    if len(sys.argv) < 5:
        print("Usage: spawn_turtlebot.py <model> <x> <y> <z>")
        return 1
    
    model = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]
    z = sys.argv[4]
    
    # Get URDF path
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(pkg_turtlebot3_description, 'urdf', f'turtlebot3_{model}.urdf')
    sdf_path = f'/tmp/robot_{model}.sdf'
    
    # Wait for Gazebo to be ready and check service availability
    # Detect the world name from available services
    print("Waiting for Gazebo to be ready...")
    max_retries = 10
    world_name = 'empty'  # Default world name from empty.sdf
    for i in range(max_retries):
        time.sleep(1)
        # Check if Gazebo service is available and detect world name
        result = subprocess.run(['gz', 'service', '-l'], capture_output=True, text=True)
        # Try to find world name from services
        world_match = re.search(r'/world/(\w+)/create', result.stdout)
        if world_match:
            world_name = world_match.group(1)
            print(f"Detected world name: {world_name}")
            print("Gazebo service is ready!")
            break
        if result.returncode == 0 and '/world/' in result.stdout:
            print("Gazebo service is ready!")
            break
        print(f"Waiting for Gazebo... (attempt {i+1}/{max_retries})")
    
    # Verify URDF file exists
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found: {urdf_path}")
        return 1
    print(f"Found URDF file: {urdf_path}")
    
    # Convert URDF to SDF using gz sdf -p (print/convert)
    print(f"Converting URDF to SDF...")
    with open(sdf_path, 'w') as f:
        result = subprocess.run(['gz', 'sdf', '-p', urdf_path], stdout=f, stderr=subprocess.PIPE, text=True)
        if result.returncode != 0:
            print(f"ERROR: URDF to SDF conversion failed: {result.stderr}")
            return 1
    
    if not os.path.exists(sdf_path) or os.path.getsize(sdf_path) == 0:
        print(f"ERROR: SDF file not created or is empty: {sdf_path}")
        return 1
    print(f"SDF file created: {sdf_path}")
    
    # Spawn entity - try using absolute file path
    abs_sdf_path = os.path.abspath(sdf_path)
    spawn_req = f'sdf_filename: "file://{abs_sdf_path}" name: "turtlebot3_{model}" pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
    
    print(f"Spawning robot at ({x}, {y}, {z})...")
    print(f"Using SDF: {abs_sdf_path}")
    
    spawn_cmd = [
        'gz', 'service', '-s', f'/world/{world_name}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '10000',
        '--req', spawn_req
    ]
    
    print(f"Running command: {' '.join(spawn_cmd)}")
    sys.stdout.flush()  # Ensure output is visible
    result = subprocess.run(spawn_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    
    if result.returncode != 0:
        print(f"ERROR: Spawn failed (return code {result.returncode})")
        print(f"STDERR: {result.stderr}")
        print(f"STDOUT: {result.stdout}")
        sys.stdout.flush()
        # Try alternative: reading SDF content and embedding it
        print("Attempting alternative spawn method with embedded SDF...")
        sys.stdout.flush()
        with open(sdf_path, 'r') as f:
            sdf_content = f.read().replace('\n', ' ').replace('"', '\\"')
        spawn_req_alt = f'sdf: "{sdf_content}" name: "turtlebot3_{model}" pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
        spawn_cmd_alt = [
            'gz', 'service', '-s', f'/world/{world_name}/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '10000',
            '--req', spawn_req_alt
        ]
        result2 = subprocess.run(spawn_cmd_alt, stderr=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        if result2.returncode != 0:
            print(f"ERROR: Alternative method also failed: {result2.stderr}")
            sys.stdout.flush()
            return 1
        else:
            print("SUCCESS: Alternative spawn method succeeded!")
            sys.stdout.flush()
    else:
        print(f"SUCCESS: Robot spawned! Service response: {result.stdout}")
        sys.stdout.flush()
    
    # Verify the entity exists in Gazebo
    print("Verifying robot entity in Gazebo...")
    sys.stdout.flush()
    time.sleep(1)  # Give Gazebo time to process
    check_cmd = ['gz', 'service', '-s', f'/world/{world_name}/state', '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.SerializedState', '--timeout', '2000', '--req', '']
    check_result = subprocess.run(['gz', 'model', '-l'], capture_output=True, text=True, timeout=5)
    if f'turtlebot3_{model}' in check_result.stdout:
        print(f"VERIFIED: Robot 'turtlebot3_{model}' found in Gazebo world!")
    else:
        print(f"WARNING: Robot may not have spawned. Available models: {check_result.stdout[:200]}")
    sys.stdout.flush()
    
    print("Robot spawn process completed!")
    return 0

if __name__ == '__main__':
    sys.exit(main())

