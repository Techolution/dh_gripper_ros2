# DH Gripper ROS 2 (Humble) Package

## Overview

This repository provides ROS 2 Humble packages for controlling DH Robotics grippers (including PGC140, AG95, DH3, RGI, etc.), their URDFs, and simulation/visualization support.

---

## Installation

1. **Install ROS 2 Humble**  
   Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

2. **Clone this repository into your ROS 2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone <this-repo-url>
   ```

3. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-gazebo-ros
   ```

4. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

5. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

---

## Usage

### 1. Launch the Gripper Driver

To start the gripper driver node (example for PGC140):

```bash
ros2 launch dh_gripper_driver dh_gripper.launch.py
```

You can override parameters:

```bash
ros2 launch dh_gripper_driver dh_gripper.launch.py Gripper_ID:=1 Gripper_Model:=PGC140 Connect_port:=/dev/ttyUSB0 BaudRate:=115200
```

### 2. Send Commands to the Gripper

Publish a control command (example):

```bash
ros2 topic pub /gripper/ctrl dh_gripper_msgs/msg/GripperCtrl "{initialize: false, position: 1000.0, force: 100.0, speed: 100.0}"
```

### 3. Visualize in RViz2

To visualize the gripper and its state, use the appropriate launch file from the URDF or description package, or create your own using `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.

---

## Notes

- All launch files are now in ROS 2 Python format (`.launch.py`).
- All parameters must match the names expected in the driver code (`Gripper_ID`, `Gripper_Model`, `Connect_port`, `BaudRate`).
- The driver expects `Gripper_ID` and `BaudRate` as integers, others as strings.
- For simulation or visualization, see the `dh_pgc140_urdf`, `dh3_urdf`, or other URDF packages.

---

## Troubleshooting

- If you get a parameter type error, ensure your launch file and C++ code use the same types for each parameter.
- Always source your workspace after building: `source install/setup.bash`
- Use `colcon build --symlink-install` for development.

---

## License

BSD

---

## Authors

- Kartik Soni

