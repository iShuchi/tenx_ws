# **Workspace for Waypoint Navigation**

This repository provides a navigation framework for the **TurtleBot3** using ROS2 Humble and Gazebo Classic. workspace integrates with the **Cartographer** package for Simultaneous Localization and Mapping (SLAM) and **Navigation2** for path planning and execution. The map saved inside folder is *custom generated* with the following commands,

   ```bash
   ros2 launch turtlebot3_cartographer cartographer_launch.py
   ros2 run teleop_twist_joy teleop_node
   ros2 run nav2_map_server map_saver_cli
   ```


## **Requirements**

Ensure following dependencies installed on the system:

### **System Requirements:**

* ROS2 Humble
* Gazebo Classic
* TurtleBot3 and TurtleBot3 simulations *(built from source)*

### **Required ROS2 Packages:**

1. **Gazebo Packages**:

   ```bash
   sudo apt install ros-humble-gazebo-*
   ```

2. **Cartographer** (For SLAM):

   * Run the following commands to install Cartographer:

   ```bash
   sudo apt install ros-humble-cartographer
   sudo apt install ros-humble-cartographer-ros
   ```

3. **Navigation2** (For Path Planning):

   * Run the following commands to install Navigation2:

   ```bash
   sudo apt install ros-humble-navigation2
   sudo apt install ros-humble-nav2-bringup
   ```

---

## **Setup Instructions**

### **Step 1: Install Dependencies**

1. Clone this repository to your workspace:

   ```bash
   git clone https://github.com/iShuchi/tenx_ws.git
   cd tenx_ws
   ```

2. Install any additional dependencies required by the workspace:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   colcon build --symlink-install
   ```

4. Add following commands at the end of .bashrc:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/tenx_ws/install/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ```

---

## **Execution Instructions**

To launch the robot simulation, **source terminal** and then follow these steps:

1. **Launch Gazebo** with TurtleBot3 simulation:

   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch the navigation stack** with pre-configured parameters:

   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/path/to/map.yaml
   ```
3. **Launch the path tracking node**:

   ```bash
   ros2 launch navigation path_tracking.launch.py
   ```
4. **Publish the waypoints**:

   ```bash
   bash ~/tenx_ws/src/scripts/waypoints.sh
   ```
---

## **Design Choices and Algorithms**

### **Path Following:**

The system uses a **Pure Pursuit** algorithm for path following, combined with a smoothing technique.

#### **Pure Pursuit Controller**:

Pure Pursuit is a geometrical path-following algorithm used to navigate the robot along a desired path. It calculates the steering angle required to move the robot towards a point on the path at a fixed lookahead distance. The controller adjusts this lookahead point dynamically to ensure smoother turns.

Mathematically, the lookahead distance ( L ) is chosen such that L = kd. Where ( k ) is a constant factor, and ( d ) is the current robot speed. The steering angle ( delta ) can be computed as atan2(2L/r).

#### **Smoothing Techniques**:

Three smoothing algorithms are used:

1. **Savitzky-Golay Smoother (SavGol)**: A polynomial filter that smooths the path while preserving the signal's characteristics, which is useful when working with noisy data.
2. **Simple Smoother**: A basic algorithm that averages the points in the path to reduce sharp turns.
3. **Pure Pursuit Smoother**: This helps smooth the desired trajectory to ensure less abrupt turns during path following.

### **Why These Algorithms?**

* **Pure Pursuit** is a widely used method for real-time control in mobile robots due to its simplicity and effectiveness in following curved paths.
* **SavGol** smoothing provides optimal results in situations where path data might be noisy or when sharp corners need to be handled gently.

### **Trapezoidal Velocity Profile for Real Robot Extension:**

For real robots, constant velocity paths are not ideal due to physical constraints, like acceleration and deceleration limits. To address this, we use a **Trapezoidal Velocity Profile** to allow for smooth acceleration and deceleration phases in the robot's motion.

The profile can be defined on the basis of acceleration, constant and deacceleration phase. This technique can be added by adjusting the velocity commands in the path-following logic. The velocity should change based on the robot's current speed and distance from the next goal, ensuring a smooth transition between states.

---

## **Extending to Real Robots**

To extend the system to a **real robot**, the following changes are required:

1. **Implement a Trapezoidal Velocity Profile**: Replace the constant velocity commands with the trapezoidal velocity profile to simulate realistic robot motion.
2. **Sensor Integration**: Use LIDAR or other sensors for real-time feedback to ensure localization and path correction.

---

## **Obstacle Avoidance Extension**

To incorporate **obstacle avoidance**, you can modify the existing `path_tracking_node.cpp` to publish a path instead of command velocities. Here's how:

### **Changes to Path Tracking Node**:

1. **Modify `path_tracking_node.cpp`** to:

   * Publish the planned path instead of velocity commands.
   * Ensure the robot follows the path published by the motion planning module.

2. **Modify the Launch File**:

   * Update the launch files to ensure the **Nav2 local DWB (Dynamic Window Approach) planner** uses the path as input.

3. **Configure the Nav2 Controller**:

   * Modify the `params.yaml` inside *config* to update the parameters for the Nav2 DWB planner to follow the path generated by your algorithm and to perform dynamic obstacle avoidance.

4. **Use Local Planners**:

   * The **Nav2 DWB planner** can be configured in the `yaml` configuration files to adjust how velocities are computed based on the robot's state and path.

---

## **AI Tools Used**

This project leverages **ROS2 Navigation2** for path planning and **Cartographer** for simultaneous localization and mapping (SLAM). Both tools provide an essential foundation for autonomous navigation in complex environments.

### **Navigation2**:

* **DWB Local Planner**: Used for real-time velocity generation while avoiding obstacles.
* **Costmaps**: For managing obstacles in the robot’s environment and ensuring the robot stays clear of them.

### **Cartographer**:

* **SLAM**: Cartographer is responsible for mapping the environment and localizing the robot in real-time.

---
