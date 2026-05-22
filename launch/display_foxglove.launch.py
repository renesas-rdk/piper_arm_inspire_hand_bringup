# *********************************************************************************************************************
# Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
#
# The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
# and/or its licensors ("Renesas") and subject to statutory and contractual protections.
#
# Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
# display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
# purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
# SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
# NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
# OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
# be subject to different terms.
# *********************************************************************************************************************

"""
Launch file for displaying Agilex Piper Arm with Inspire RH56 Hand in Foxglove Studio.

This launch file starts:
- robot_state_publisher: Publishes TF transforms from URDF
- joint_state_publisher: Publishes joint states with configurable initial positions
- foxglove_bridge: WebSocket bridge for Foxglove Studio visualization

Usage:
  ros2 launch piper_arm_inspire_hand_bringup display_foxglove.launch.py
  ros2 launch piper_arm_inspire_hand_bringup display_foxglove.launch.py hand_side:=left
  ros2 launch piper_arm_inspire_hand_bringup display_foxglove.launch.py hand_side:=right

Then connect Foxglove Studio to ws://<foxglove_bridge_ip>:8765

Arguments:
  hand_side: Which hand to use ('left' or 'right'). Default: 'left'
"""

import os
from typing import List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function to evaluate launch configurations at runtime."""
    package_name = 'piper_arm_inspire_hand_bringup'

    # Get the hand_side value at runtime
    hand_side_value = LaunchConfiguration('hand_side').perform(context)

    # Paths
    pkg_share = get_package_share_directory(package_name)

    # Select and process the appropriate XACRO file
    if hand_side_value == 'left':
        xacro_file = os.path.join(pkg_share, 'urdf', 'piper_arm_inspire_hand_left.urdf.xacro')
    else:
        xacro_file = os.path.join(pkg_share, 'urdf', 'piper_arm_inspire_hand_right.urdf.xacro')

    robot_description = xacro.process_file(xacro_file).toxml()

    foxglove_bridge_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    # Nodes
    nodes: List = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),
        # Joint state publisher (same for both hands)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'zeros.joint2': 0.0,
                'zeros.thumb_proximal_yaw_joint': 1.308,
                'zeros.thumb_proximal_pitch_joint': 0.3,
                'zeros.index_proximal_joint': 0.75,
                'zeros.middle_proximal_joint': 0.75,
                'zeros.ring_proximal_joint': 1.47,
                'zeros.pinky_proximal_joint': 1.47
            }]
        ),
        # Foxglove bridge for web-based visualization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_bridge_launch)
        )
    ]

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Agilex Piper Arm with Inspire Hand Foxglove visualization."""
    # Launch arguments
    hand_side_arg = DeclareLaunchArgument(
        'hand_side',
        default_value='left',
        description='Which hand to use: left or right',
        choices=['left', 'right']
    )

    return LaunchDescription([
        hand_side_arg,
        OpaqueFunction(function=launch_setup)
    ])
