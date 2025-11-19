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

* World/turtlebot3_burger/a__namespace_base_footprint   <-- default articulation root (undesired)


Important: Isaac Sim sometimes sets an extra child prim (e.g. a__namespace_base_footprint) as the articulation root. For correct physics and control you should move the articulation root to the robot top-level prim:

* World/turtlebot3_burger   ← set this as articulation root


If the articulation root is left at World/turtlebot3_burger/a__namespace_base_footprint you may have problems with moving the robot from ROS /cmd_vel or with proper TF/odom behavior.

# ActionGraphs in Isaac sim
Action Graphs in Isaac Sim are node-based visual programming graphs that let you connect simulation events, ROS2 messages, physics, and robot behaviors without writing code.
 
* we can create Action graphs by clicking Windows ---> graph editor ----> Action Graphs. here you can create action graphs.
* I created these Action Graphs
    * articulation_controller
    * differential_controller
    * on_playback_tick
    * ros2_context
    * ros2_subscribe_twist
    * scale_to_from_stage_units
    * ros2_publish_odometry
    * ros2_publish_joint_state
    * break_3_vector
    * break_3_vector_01
    * constant_token
    * constant_token_01
    * make_array
    * isaac_compute_odometry_node
    * isaac_read_simulation_time
    * ros2_publish_raw_transform_tree
    * ros2_publish_clock
 
<img width="2552" height="1433" alt="image" src="https://github.com/user-attachments/assets/65a898b3-13c4-436f-85bc-52e30646a1de" />


It enables my TurtleBot3 robot in Isaac Sim to be fully controlled from ROS2.
With this Action Graph, the robot can:

* receive /cmd_vel velocity commands from my ROS2 controller,
* move its wheels using differential drive,
* publish odometry (/odom) back to ROS2,
* publish joint states and TF frames (/joint_states, /tf),
* and stay time-synchronized with ROS2 through the /clock topic.

In simple terms: this Action Graph connects Isaac Sim and ROS2, so the robot can move, publish odometry, and work with my controller pipeline.

#

   

