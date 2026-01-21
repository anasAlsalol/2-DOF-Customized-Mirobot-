#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamics_control',
            executable='unified_dynamics_node',
            name='unified_dynamics_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Robot parameters
                'robot_name': 'planar_2dof',
                'joint_names': ['joint1', 'joint2'],
                'link_lengths': [0.3, 0.3],
                'masses': [1.0, 1.0],
                'gravity': 9.81,
                
                # Control parameters
                'control_frequency': 100.0,
                'control_mode': 'external_ros',
                'use_lagrangian': True,
                'use_newton_euler': True,
                'kp': [150.0, 120.0],
                'kd': [25.0, 20.0],
                'max_torque': 50.0,
                
                # Trajectory
                'trajectory_type': 'sine',
                
                # ROS topics (MUST MATCH LUA SCRIPT)
                'joint_state_topic': '/robot/joint_states',
                'torque_command_topic': '/robot/joint_torque_cmd',
                'reference_trajectory_topic': '/robot/reference_trajectory',
                'control_mode_topic': '/sim/control_mode',
                'heartbeat_topic': '/ros/heartbeat',
                'sim_status_topic': '/sim/status',
                'handshake_topic': '/dynamics/handshake',
                
                # Data logging
                'log_data': True,
                'log_directory': './logs',
                'plot_realtime': True,
                'save_interval': 10.0,
                
                # Connection monitoring
                'connection_timeout': 2.0
            }]
        )
    ])
