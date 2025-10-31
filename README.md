# Custom DWA Planner

A custom Dynamic Window Apprach (DWA) implementation for ROS2 Humble with TurtleBot3 in Gazebo

## Quick Start

Please follow the instructions below. **Skip to Step 3 if you have a working ROS2 Humble and Gazebo setup.**

## Step 1: Install ROS2 Humble
Follow the official installation guide:
https://docs.ros.org/en/humble/Installation.html

## Step 2: Install Gazebo
Official Gazebo installation guide: 
https://gazebosim.org/docs/garden/install_ubuntu/

For Turtlebot3 Gazebo simulation:
```
sudo apt install ros-humble-turtlebot3-gazebo -y
```

## Step 3: Clone repository and build
```
mkdir -p ~/dwa_ws/src
cd ~/dwa_ws/src
git clone https://github.com/Mayankpr04/custom_DWA_ROS2.git
```
```
cd ~/dwa_ws
colcon build
```
After you build, do not forget to source. A good practice is to run ``` source install/setup.bash ``` everytime you enter the directory.

## Step 4: Launch / run
After you build, run the following to launch the DWA planner
```
cd ~/dwa_ws
source install/setup.bash
ros2 launch custom_dwa_planner dwa_planner.launch.py
```
First, the Gazebo world will spawn with the TurtleBot3. Wait for a few seconds, and an RViz window will appear.

 ## Visualization
 You should be able to see something like this:
 
![Demo - Made with Clipchamp](https://github.com/user-attachments/assets/9f8d724b-a6d0-43b0-bb02-e9535c0f7d55)

<img width="424" height="240" alt="Screenshot from 2025-10-31 14-42-17" src="https://github.com/user-attachments/assets/f492d9a3-dafa-485d-bb4f-528a1924f4dc" />
<img width="424" height="240" alt="Screenshot from 2025-10-31 14-41-53" src="https://github.com/user-attachments/assets/1097f46a-714f-4937-a81a-9796635d8817" />
The RViz window shows the path taken, where the goal point is, along with the laser scan for visual information. Please note that while the above run works as shown,
sometimes the path taken is eccentric at the beginning, I am unsure of why that happens, but it does navigate to the goal point




