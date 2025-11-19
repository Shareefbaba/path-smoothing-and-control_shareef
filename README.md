# Project Overview

This project focuses on designing and implementing a complete trajectory tracking pipeline for a differential-drive mobile robot inside NVIDIA Isaac Sim, integrated seamlessly with ROS2 Humble. The goal was to generate smooth motion trajectories, visualize them, and control the robot to accurately follow the planned path in a simulated environment.

# SOftware Configurations
## System Requirements & Dependencies

- **Ubuntu 22.04**
- **ROS2 Humble** installed and sourced
- **Isaac Sim version 5.0.0**
  - ROS2 Bridge extension enabled

# Robot Setup in Isaac sim
In Isaac Sim I used the Warehouse environment and imported the TurtleBot3 Burger URDF.
You can find the URDF used here in the repo:
[Turtlebot3_burger_urdf](isaacsim/urdf/turtlebot3_burger.urdf). 
When importing, make sure the base is movable and the robot accepts velocity commands.

Now prim path looks like World/turtlebot3_burger and child path follows

<img width="2560" height="1438" alt="image" src="https://github.com/user-attachments/assets/4ed3143a-7a95-43cc-8013-c5ce75fee204" />
Scene prims and articulation root (important)

After import the robot prim looks like:

World/turtlebot3_burger
└─ a__namespace_base_footprint   <-- default articulation root (undesired)


Important: Isaac Sim sometimes sets an extra child prim (e.g. a__namespace_base_footprint) as the articulation root. For correct physics and control you should move the articulation root to the robot top-level prim:

World/turtlebot3_burger   ← set this as articulation root


If the articulation root is left at World/turtlebot3_burger/a__namespace_base_footprint you may have problems with moving the robot from ROS /cmd_vel or with proper TF/odom behavior.
