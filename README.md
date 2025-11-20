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

# path Smoothing and time-parameterization
Path smoothing takes the raw discrete waypoints and converts them into a smooth, continuous curve that the robot can follow without sudden turns or sharp changes. In my project, I used the "Centripetal Catmull–Rom Spline", which naturally passes through all waypoints and produces a smooth path with controlled curvature.

Once the smooth path is generated, time-parameterization is applied. This assigns a timestamp to every point along the path based on a chosen velocity. It converts the geometric path into a time-based trajectory, where each point has:

(x, y, t)


This makes it possible for the controller to follow the path at a consistent speed and compute how fast the robot should move at each step.

In simple terms:
Path smoothing makes the path smooth, and time-parameterization assigns time to each point so the robot knows when to be where. 
[the codes are availabe here ](tenx_assignment/utils).

and can you see the plot it easily understandable below image
<img width="1005" height="501" alt="Screenshot from 2025-11-19 12-47-42" src="https://github.com/user-attachments/assets/b4801066-d2ba-4638-8ab9-39e9c9fb0541" />  <img width="1010" height="483" alt="Screenshot from 2025-11-20 02-00-12" src="https://github.com/user-attachments/assets/f3f0d143-089f-4287-8b39-85c799aacbbe" />

# ROS2 Nodes for Path Smoothing, Trajectory Generation, and Control
ROS2 Nodes Overview

To run the full motion pipeline, I created several ROS2 nodes. Each node is modular and handles one specific part of the project: smoothing the raw path, generating a time-based trajectory, and controlling the robot to follow it.

1. Path Smoother Node

[path_smoother_node](tenx_assignment/nodes/path_smoother_node.py)
This node takes the raw 2D waypoints and applies the Centripetal Catmull–Rom Spline to generate a smooth path.
It publishes the smoothed path as a list of (x, y) points.

2. Trajectory Generator Node

File: tenx_assignment/nodes/trajectory_generator_node.py

This node receives the smoothed path and performs time-parameterization.
It assigns timestamps to each point and publishes a time-based trajectory in the form:

(x, y, t)

3. Trajectory Controller Node

[trajectory_generator_node](tenx_assignment/nodes/trajectory_generator_node.py)

This node reads the time-parameterized trajectory and computes the required linear and angular velocities for the robot.
It uses a Pure-Pursuit Controller, and publishes /cmd_vel that sends commands to Isaac Sim.

4. Path Publisher Node (optional for testing)

File: tenx_assignment/nodes/path_publisher_node.py
Used to publish a test set of waypoints without needing a separate script.

5. Trajectory to Path Node

[trajectory_controller_node](tenx_assignment/nodes/trajectory_controller_node.py)

Converts (x, y, t) back into a nav_msgs/Path message so you can visualize the trajectory in RViz2.

How the Nodes Work Together 

raw_waypoints  
      
     ↓
Path Smoother Node  

     ↓ 
smoothed_path  

     ↓
Trajectory Generator Node  

     ↓
time_parameterized_trajectory 

     ↓
Controller Node  

     ↓
/cmd_vel → Isaac Sim Robot

# Launch File

[Launch File](tenx_assignment/launch/test.launch.py)

The launch file is used to start all the required ROS2 nodes for this project in a single command. Instead of running each node manually, the launch file groups them together and handles all parameters, topic names, and node startup order.

Purpose of the Launch File

The launch file:

* Starts the Path Smoother Node

* Starts the Trajectory Generator Node

* Starts the Tracking Controller Node

* Starts the Path Publisher Node (optional, for testing)

* Configures parameters (if needed)

Ensures all nodes use simulation time (use_sim_time=true)

Sets namespaces and topic remappings (if required)

In simple terms:

👉 The launch file is the central script that starts the entire pipeline:
Path Smoothing → Trajectory Generation → Trajectory Tracking.







   

