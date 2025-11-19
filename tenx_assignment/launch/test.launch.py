#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 1) Path Smoother
        Node(
            package='tenx_assignment',
            executable='path_smoother_node',
            name='path_smoother_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 2) Trajectory Generator
        Node(
            package='tenx_assignment',
            executable='trajectory_generator_node',
            name='trajectory_generator_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 3) Trajectory Controller
        Node(
            package='tenx_assignment',
            executable='trajectory_controller_node',
            name='trajectory_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

    ])
