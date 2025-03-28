# ROS 2 Navigation Stack - Harshith and Samarth 

## ✅ Features

- Robot description using **URDF** + **Xacro**
- Visualization via **RViz2**
- Static and dynamic transforms via:
   - `robot_state_publisher`
   - `joint_state_publisher`
   - Optional `joint_state_publisher_gui`
- Basic kinematics using `diff_drive_controller`
- Simple launch files for easy simulation setup

---

## 🗂 Folder Structure

```
ros_navigation_stack/
├── my_robot_description/    # Contains URDF/Xacro and RViz configs
├── my_robot_bringup/        # Launch files to bring up the robot simulation
├── my_robot_control/        # Controller configuration (diff_drive_controller)
├── CMakeLists.txt
├── package.xml
└── ...
```

---

## 🚀 Requirements

- ROS 2 Foxy / Galactic / Humble
- `joint_state_publisher_gui` (optional, for GUI joint slider)
- `ros2_control`
- `diff_drive_controller`

---

## ⚙️ Installation

```bash
# Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/sharshith1312/ros_navigation_stack.git

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash  # or source install/setup.zsh
```

---

## 🟣 Running the Simulation

```bash
# Launch the robot in RViz
ros2 launch my_robot_bringup bringup.launch.py
```

---

## 🟢 Notes
- Make sure you have `joint_state_publisher_gui` installed if you want to use the GUI sliders:
   ```bash
   sudo apt install ros-foxy-joint-state-publisher-gui
   ```
   On macOS or source builds, you may have to build it manually.
   
- If you're using a different ROS 2 version, adjust package versions accordingly.
