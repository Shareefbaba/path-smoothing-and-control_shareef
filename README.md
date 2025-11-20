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

# Simulation 

To run the simulation, I first open Isaac Sim and load the Warehouse environment. After that, I press Play to start physics, because the robot will only respond to ROS2 commands when the physics engine is running.

Once physics is active, I check whether Isaac Sim is publishing ROS2 topics correctly. I open a terminal and run:

ros2 topic list


If I can see topics like /odom, /joint_states, /tf, and /clock, it confirms that the ROS2 Bridge extension inside Isaac Sim is working properly.

Next, I build my ROS2 workspace:

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash


After sourcing the workspace, I launch the entire pipeline using my launch file:

ros2 launch tenx_assignment test.launch.py


This automatically starts the path smoother, trajectory generator, and controller. At this point, I can switch back to Isaac Sim and see the robot begin to move along the planned path.

<img width="2560" height="1440" alt="Screenshot from 2025-11-20 05-45-07" src="https://github.com/user-attachments/assets/e7c49c3d-3596-477c-8d68-fcd7a09bd401" />

<img width="2560" height="1440" alt="Screenshot from 2025-11-20 05-45-12" src="https://github.com/user-attachments/assets/321b4bbe-e27e-4b3c-9f5d-6bf3d66467b1" />


<img width="2560" height="1440" alt="image" src="https://github.com/user-attachments/assets/b958dc1e-fdf6-4ba1-9de8-0c4699d1c401" />


To visualize the results more clearly, I also open RViz2:

rviz2


In RViz, I add displays for /path_smoothed, /trajectory, /odom, and /tf.
This lets me monitor the smoothed path, the time-parameterized trajectory, and the robot’s actual movement.


<img width="2560" height="1440" alt="Screenshot from 2025-11-19 03-22-03" src="https://github.com/user-attachments/assets/f074cd4e-9b43-4cc6-9d53-ade8752fd49a" />

<img width="2560" height="1440" alt="Screenshot from 2025-11-19 03-02-46" src="https://github.com/user-attachments/assets/ed63dc5c-ada6-452a-9a97-39d022ce9e55" />

# My design choices, algorithms, architectural decisions

## Design Choices

For this project, I tried to keep the overall structure simple and easy to work with. I separated each part of the system into its own ROS2 node so that path smoothing, trajectory generation, and control would not interfere with each other. This made it easier to test each piece on its own. I also chose Python nodes because it allowed me to make quick changes and rebuild instantly, which saved a lot of time during development. Isaac Sim handled all the low-level simulation and physics, while ROS2 handled the logic, so the two sides stayed cleanly separated.

## Algorithms

For smoothing the waypoints, I used the Centripetal Catmull–Rom spline because it produces a smooth curve that passes directly through the original points without creating sharp bends. After smoothing the path, I used a simple time-parameterization approach based on the path length, which lets me assign a timestamp to every point on the path. This converts the path into a usable trajectory. For tracking, I chose a Pure-Pursuit style controller. It’s straightforward, works well for differential-drive robots, and is easy to tune. The controller looks ahead on the trajectory and generates the linear and angular velocity commands needed to follow it.

## Architectural Decisions

I designed the system so each ROS2 node has one clear job. The smoother publishes a clean path, the generator adds timing information, and the controller reads the trajectory together with odometry from Isaac Sim and sends /cmd_vel commands back. Isaac Sim’s ROS2 bridge takes care of publishing /odom, /tf, and /clock, so everything stays synchronized with the simulator. The launch file puts all nodes together and makes the whole pipeline easy to run. This structure makes the system stable, easy to understand, and easy to extend later if I want to add obstacle avoidance or switch to a different controller.

# extend this to a real robot

To extend this system from simulation to a real autonomous robot, I would keep the same overall ROS2 pipeline but replace the simulated components with real hardware and sensors. Instead of Isaac Sim providing odometry and wheel control, the robot would use wheel encoders, an IMU, and a LiDAR or depth camera for perception and localization. These sensors would be fused using packages like robot_localization to generate stable /odom and /tf data. The /cmd_vel commands produced by my tracking controller would be sent to a real motor driver or a ros2_control velocity controller to move the robot. With these additions, the robot would be able to follow the generated trajectory fully autonomously, making decisions based on live sensor feedback and adapting its motion in real environments. Safety features like watchdog timers, speed limits, emergency stop, and proper calibration of wheel radius and wheelbase would be added to ensure reliable behavior on hardware. With these changes, the trajectory pipeline—path smoothing, time-parameterization, and control—would work the same way on a physical robot as in simulation.

# Brief on the AI tools used 

I worked closely with ChatGPT throughout this project, mainly because I had only two days to complete everything and wanted to make sure the code was clean, correct, and well structured. I wrote all the core logic myself — the path smoothing, trajectory generation, controller nodes, and launch setup — and then used ChatGPT and Grok to refine the code, fix issues faster, and improve the overall design. I’m comfortable with Linux, ROS2, and Isaac Sim, but coming from a mechanical engineering background, I’m still building my confidence in coding. Using AI helped me stay productive, avoid getting stuck, and complete the project at a much faster pace. The ideas, testing, debugging, and final integration in Isaac Sim were done by me, but AI tools acted like an assistant that helped me polish and speed up the work.

# Extra Credit: How I Would Extend This to Avoid Obstacles

To extend this system with obstacle avoidance, I would add a local planning and perception layer on top of the existing trajectory-following pipeline. The robot would use a LiDAR or depth camera to detect obstacles in real time and build a local costmap around itself. A small local planner—such as a DWA-based velocity planner or a simple reactive safety layer—would sit between the controller and the motor commands. This planner would check the planned path against incoming sensor data and adjust the robot’s velocity if an obstacle appears. If the obstacle blocks the path, the robot could slow down, stop, or re-route around it while still trying to follow the global trajectory as closely as possible. This approach works well because it preserves the global planning pipeline while adding intelligence and safety at the local level.

In a real robot, this would be implemented using LiDAR scans, depth images, or 3D point clouds to detect obstacles. Since I previously worked with LiDARs and sensors in my past job, I’m comfortable interpreting scan data, creating basic costmaps, and integrating perception into a ROS2 system. With that experience, I would use sensor fusion tools like robot_localization and then feed the fused odometry and LiDAR scans into a local planner to generate safe, collision-free /cmd_vel commands. This would allow the robot to move autonomously in more complex environments, avoid collisions, and adapt its motion based on real-time sensor input.






   

