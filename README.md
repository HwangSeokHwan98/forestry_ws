# `ecvt_controller`

This package is a ROS 2 package designed for controlling a robot in Gazebo using effort and velocity controllers. The package includes:
- A launch file for simulating the robot in Gazebo.
- ROS 2 nodes for publishing effort and velocity commands.

---

## **Installation**

1. Clone this repository into your ROS 2 workspace:
   ```bash
   mkdir rsc2_ws/src
   cd ~/rsc2_ws/src
   git clone https://github.com/joosunlee/ecvt_controller.git
   ```

2. Build the workspace:
   ```bash
   cd ~/rsc2_ws
   colcon build --packages-select ecvt_controller
   ```

3. Source the workspace:
   ```bash
   source ~/rsc2_ws/install/setup.bash
   ```


4. Folder Structure
   ```bash
   ecvt_controller/
   ├── CMakeLists.txt
   ├── package.xml
   ├── launch/
   │   └── e_ecvt_gazebo_v2.launch.py   # Gazebo launch file
   ├── src/
   │   └── ecvt_effort_controller.cpp  # Node for publishing effort commands
   ├── urdf/
   │   └── e_ecvt_v2.xacro.urdf        # XACRO-based URDF file
   ├── meshes/
   │   └── <.stl>            # Mesh files used in URDF
   ├── config/
   │   └── <.yaml>        # Configuration files for controllers
   └── README.md                       # This file
   ```

![Gazebo](Step1.gif)
## Gazebo Simulation
1. To start the Gazebo simulation, run:
   ```bash
   ros2 launch ecvt_controller e_ecvt_gazebo_v2.launch.py
   ```

![Gazebo](Gazebo_under_effort.gif)
## Gazebo Controller
1. Effort Controller
- The effort controller manages the lower part of the robot. Publish using:


   ```bash
   ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, -70000.0, 0.0, 0.0, 0.0, -70000.0, 0.0, 0.0, 0.0, 50000, 0.0, 0.0, 0.0, 50000.0, 0.0, 0.0]"
   ```

2. Velocity Controller
- The velocity controller manages the robot's wheels. Publish using:

   ```bash
   ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"
   ```


## Troubleshooting

### Fixing `mesh_prefix` in URDF File
If you encounter an error related to the `mesh_prefix`, you need to edit the `ecvt_controller/urdf/e_ecvt_v2.xacro.urdf` file and set the correct path for your system. Update the `mesh_prefix` property as follows:

   ```bash
   <xacro:property name="mesh_prefix" value="file:///home/<username>/rsc2_ws/src/ecvt_controller/meshes/" />
   ```
Replace <username> with your actual username and ensure the path matches your system's environment.
