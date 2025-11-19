#!/usr/bin/env python3
"""
Dev-friendly launch that runs the node scripts directly (no need for installed entrypoints).
It launches in order:
  1) path_smoother_node.py
  2) trajectory_generator_node.py
  3) trajectory_controller_node.py

Assumes the scripts live in:
  ~/ros2_ws/src/tenx_assignment/tenx_assignment/nodes
Adjust SCRIPT_DIR if your scripts are elsewhere.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
SCRIPT_DIR = os.path.expanduser('~/ros2_ws/src/tenx_assignment/tenx_assignment/nodes')

def generate_launch_description():
    path_smoother = ExecuteProcess(cmd=['python3', os.path.join(SCRIPT_DIR, 'path_smoother_node.py')], output='screen')
    traj_gen = ExecuteProcess(cmd=['python3', os.path.join(SCRIPT_DIR, 'trajectory_generator_node.py')], output='screen')
    traj_ctrl = ExecuteProcess(cmd=['python3', os.path.join(SCRIPT_DIR, 'trajectory_controller_node.py')], output='screen')

    ld = LaunchDescription()
    ld.add_action(LogInfo(msg='[test.launch] starting path_smoother_node (immediate)'))
    ld.add_action(path_smoother)
    # small delay to let smoother publish
    ld.add_action(TimerAction(period=0.6, actions=[LogInfo(msg='[test.launch] starting trajectory_generator_node'), traj_gen]))
    # start controller later so it can receive latched messages
    ld.add_action(TimerAction(period=1.5, actions=[LogInfo(msg='[test.launch] starting trajectory_controller_node'), traj_ctrl]))
    return ld

if __name__ == '__main__':
    # allow direct 'python3 test.launch.py' execution for debugging
    import launch
    ls = launch.LaunchService()
    ls.include_launch_description(launch.LaunchDescriptionSource(generate_launch_description()))
    ls.run()
