# ROS 2 URDF Modeling of a 4-DOF Industrial Robotic Arm

## 📌 Project Overview

This project focuses on designing and modeling a four-degree-of-freedom (4-DOF) industrial robotic arm using the Unified Robot Description Format (URDF/Xacro) in ROS 2 Humble. It defines the robot's links, joints, motion limits, visual geometry, collision geometry, and inertial properties.

The project includes a ROS 2 package, launch files, an RViz2 configuration, and a validation script to support robot model inspection and structural verification.

## Objectives

* Design the kinematic structure of a 4-DOF robotic arm.
* Define robot links and joints using URDF/Xacro.
* Configure revolute joint axes and motion limits.
* Specify visual, collision, and inertial properties.
* Prepare a launch configuration for RViz2 visualization.
* Validate the robot model and its link-joint structure.
* Test the package build using GitHub Actions.

## 🛠️ Technologies Used

* **ROS 2 Humble:** Robot software framework
* **URDF/Xacro:** Robot description and modeling
* **RViz2:** Robot visualization
* **robot_state_publisher:** Publishes robot link transforms
* **joint_state_publisher_gui:** Allows interactive joint manipulation
* **Python:** Validation utilities and package configuration
* **GitHub Actions:** Automated cloud build testing

##  Robot Structure

The robot model contains six links connected by five joints.

### Links

| Link Name        | Description                     |
| ---------------- | ------------------------------- |
| `base_link`      | Robot base                      |
| `shoulder_link`  | Shoulder assembly               |
| `upper_arm_link` | Upper arm                       |
| `forearm_link`   | Forearm section                 |
| `wrist_link`     | Wrist assembly                  |
| `tool_link`      | End-effector or tool attachment |

### Joints

| Joint Name         | Joint Type | Parent Link      | Child Link       |
| ------------------ | ---------- | ---------------- | ---------------- |
| `joint_1`          | Revolute   | `base_link`      | `shoulder_link`  |
| `joint_2`          | Revolute   | `shoulder_link`  | `upper_arm_link` |
| `joint_3`          | Revolute   | `upper_arm_link` | `forearm_link`   |
| `joint_4`          | Revolute   | `forearm_link`   | `wrist_link`     |
| `tool_fixed_joint` | Fixed      | `wrist_link`     | `tool_link`      |

The model has four degrees of freedom because it contains four revolute joints. The fixed joint attaches the tool without adding another degree of freedom.

## Project Structure

```text
ROS2-URDF-4DOF-Arm/
├── .github/
│   └── workflows/
│       └── ros2-build.yml
└── four_dof_arm_description/
    ├── four_dof_arm_description/
    │   ├── __init__.py
    │   └── validate_urdf.py
    ├── launch/
    │   └── display.launch.py
    ├── resource/
    │   └── four_dof_arm_description
    ├── rviz/
    │   └── display.rviz
    ├── urdf/
    │   └── four_dof_arm.urdf.xacro
    ├── docs/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── REPORT.md
```

## ⚙️ Requirements

The intended environment is Ubuntu 22.04 with ROS 2 Humble.

Required components include:

* `ros-humble-desktop`
* `xacro`
* `robot_state_publisher`
* `joint_state_publisher_gui`
* `rviz2`
* `colcon`

## 🚀 Installation and Execution

### 1. Source ROS 2

Open an Ubuntu terminal and run:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  python3-colcon-common-extensions
```

### 3. Build the Package

Place the `four_dof_arm_description` package inside a ROS 2 workspace's `src` directory.

For example:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Copy the package into this directory, then run:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4. Launch the Robot Model

```bash
ros2 launch four_dof_arm_description display.launch.py
```

If the launch succeeds, RViz2 should open with the robot model and joint controls.

**Note:** These commands describe the intended workflow. They must be executed in a compatible ROS 2 environment to confirm that the package builds and the visualization works.

## Validation and Testing

The project includes a Python validation script at:

```text
four_dof_arm_description/validate_urdf.py
```

The validation process should check:

* Robot link and joint definitions
* Joint types and axes
* Joint motion limits
* Visual and collision geometry
* Link-joint connectivity
* URDF/Xacro structure
* Package build status
* RViz2 visualization
* TF tree connectivity

Successful build, visualization, and TF verification should be recorded only after the corresponding tests have actually been executed.

## Demonstration Evidence

The final project submission should include:

* RViz2 screenshot showing the robotic arm
* Joint control demonstration
* TF tree screenshot or generated TF diagram
* Successful package build output
* Completed project report in PDF format

## 📊 Project Status

The source files and project configuration have been prepared. The ROS 2 package build, RViz2 visualization, and TF verification require execution in a compatible ROS 2 environment.

## Author

**Radharani Pradhan**

Project: URDF Modeling of a 4-DOF Industrial Robotic Arm in ROS 2

## 📄 License

This project is intended for educational and internship purposes.
