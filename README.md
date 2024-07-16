---

# Robot Arm Control Project

## Overview

The Robot Arm Control Project demonstrates the implementation of a robotic arm control system using the Robot Operating System (ROS) and MoveIt for motion planning and control. This project includes configurations for joint state publishers and MoveIt setups to facilitate the simulation and control of a robot arm in a virtual environment.

## Prerequisites

Before setting up the project, ensure you have the following installed:

- **Ubuntu 20.04**: The preferred operating system for ROS Noetic.
- **ROS Noetic**: The primary ROS distribution used.
- **MoveIt**: A ROS package for motion planning.
- **Git**: For version control.

## Project Structure

- **src**: Contains the source code for the ROS packages.
- **launch**: Contains launch files to start ROS nodes and configurations.
- **urdf**: Contains the URDF (Unified Robot Description Format) files describing the robot's model.
- **config**: Configuration files for MoveIt and other settings.

## Setup Instructions

### Step 1: Setup ROS Workspace

1. **Create and navigate to the ROS workspace directory**:
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   ```
2. **Build the workspace**:
   ```sh
   catkin_make
   source devel/setup.bash
   ```

### Step 2: Create and Build the ROS Package

1. **Navigate to the `src` directory**:
   ```sh
   cd ~/catkin_ws/src
   ```
2. **Create a new package named `robot_arm_control`**:
   ```sh
   catkin_create_pkg robot_arm_control std_msgs rospy roscpp
   ```
3. **Navigate back to the workspace and build**:
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

### Step 3: Create URDF File

1. **Create the URDF directory and file**:
   - Navigate to the package directory:
     ```sh
     cd ~/catkin_ws/src/robot_arm_control
     ```
   - Create the `urdf` directory and the URDF file:
     ```sh
     mkdir -p urdf
     touch urdf/robot_arm.urdf
     ```
2. **Add content to the `robot_arm.urdf` file**:
   ```xml
   <?xml version="1.0"?>
   <robot name="robot_arm">
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.5 0.5 0.1"/>
         </geometry>
       </visual>
     </link>
     <joint name="base_to_link1" type="fixed">
       <parent link="base_link"/>
       <child link="link1"/>
     </joint>
     <link name="link1">
       <visual>
         <geometry>
           <box size="0.1 0.1 0.5"/>
         </geometry>
       </visual>
     </link>
     <joint name="link1_to_link2" type="continuous">
       <parent link="link1"/>
       <child link="link2"/>
       <origin xyz="0 0 0.5"/>
       <axis xyz="0 1 0"/>
     </joint>
     <link name="link2">
       <visual>
         <geometry>
           <box size="0.1 0.1 0.5"/>
         </geometry>
       </visual>
     </link>
   </robot>
   ```

### Step 4: Create Launch Files

1. **Create a launch file for the joint state publisher**:
   - Navigate to the launch directory:
     ```sh
     mkdir -p ~/catkin_ws/src/robot_arm_control/launch
     ```
   - Create a file named `joint_state_publisher.launch`:
     ```xml
     <launch>
       <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
       <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
       <param name="robot_description" command="$(find xacro)/xacro $(find robot_arm_control)/urdf/robot_arm.urdf" />
     </launch>
     ```

2. **Create a launch file for MoveIt**:
   - Create a file named `moveit.launch`:
     ```xml
     <launch>
       <arg name="robot_description" command="$(find xacro)/xacro '$(find robot_arm_control)/urdf/robot_arm.urdf'" />
       <include file="$(find moveit_ros_move_group)/launch/move_group.launch" />
       <node name="move_group_interface" pkg="moveit_ros_move_group_interface" type="move_group_interface" />
     </launch>
     ```

## Running the Project

### Launch the Joint State Publisher

1. **Navigate to the workspace**:
   ```sh
   cd ~/catkin_ws
   ```
2. **Launch the joint state publisher**:
   ```sh
   roslaunch robot_arm_control joint_state_publisher.launch
   ```

### Launch MoveIt for Motion Planning

1. **Navigate to the workspace**:
   ```sh
   cd ~/catkin_ws
   ```
2. **Launch MoveIt**:
   ```sh
   roslaunch robot_arm_control moveit.launch
   ```

## Troubleshooting

- **Issue: Launch file not found**:
  Ensure that the path to the launch file is correct and that you have sourced your workspace:
  ```sh
  source ~/catkin_ws/devel/setup.bash
  ```

- **Issue: Missing dependencies**:
  Install any missing dependencies using rosdep:
  ```sh
  rosdep install --from-paths src --ignore-src -r -y
  ```

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or additions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

For any inquiries or issues, please contact:
- **mrcode10sam@gmail.com**

---
