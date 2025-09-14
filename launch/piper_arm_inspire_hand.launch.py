# ********************************************************************************************************************
# Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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
# ********************************************************************************************************************

"""
Launch file for Agilex Piper Arm with Inspire RH56 Hand integrated system.

This launch file starts:
- ros2_control_node: Main controller manager for both arm and hand hardware interfaces
- robot_state_publisher: Publishes TF transforms from combined URDF
- joint_state_broadcaster: Publishes joint states from both arm and hand hardware
- cartesian_motion_controller: Provides Cartesian space motion control for the arm
- hand_position_controller: Provides direct position commands for hand joints
- foxglove_bridge: WebSocket bridge for Foxglove Studio visualization

Usage:
  # For physical robot with CAN and serial interfaces:
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py can_interface:=can1 serial_port:=/dev/ttyUSB1

  # For right hand configuration:
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py hand_side:=right

  # For SIMULATION/TESTING without physical hardware (RECOMMENDED for testing):
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py use_mock_hardware:=true

  Then connect Foxglove Studio to ws://<foxglove_bridge_ip>:8765

Test Cartesian motion commands in another terminal with:
  ros2 topic pub -1 /agilex_piper_cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped "{
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.2, y: 0.0, z: 0.2},
      orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
    }
  }"

Test hand position commands:
  ros2 topic pub -1 /inspire_rh56_hand_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.3, 0.6, 0.0, 0.0, 1.4, 1.4]}"

Observe the integrated system moving in Foxglove Studio.

NOTE: Use 'use_mock_hardware:=true' for simulation or safe testing without physical hardware!
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


def launch_setup(context, *args, **kwargs) -> List[Node]:
    """Setup function to evaluate launch configurations at runtime."""
    # Get launch configurations
    can_interface_value = LaunchConfiguration('can_interface').perform(context)
    serial_port_value = LaunchConfiguration('serial_port').perform(context)
    baudrate_value = LaunchConfiguration('baudrate').perform(context)
    use_mock_hardware_value = LaunchConfiguration('use_mock_hardware').perform(context)
    hand_side_value = LaunchConfiguration('hand_side').perform(context)

    # Get package directories
    pkg_share = get_package_share_directory('piper_arm_inspire_hand_bringup')

    # Select and process the appropriate XACRO file based on hand side
    if hand_side_value == 'left':
        robot_description_xacro = os.path.join(
            pkg_share, 'urdf', 'piper_arm_inspire_hand_left.urdf.xacro'
        )
    else:
        robot_description_xacro = os.path.join(
            pkg_share, 'urdf', 'piper_arm_inspire_hand_right.urdf.xacro'
        )

    # Process XACRO file with parameters
    robot_description_raw = xacro.process_file(
        robot_description_xacro,
        mappings={
            'can_interface': can_interface_value,
            'serial_port': serial_port_value,
            'baudrate': baudrate_value,
            'use_mock_hardware': use_mock_hardware_value
        }
    ).toxml()

    robot_description = {'robot_description': robot_description_raw}

    # Controller configurations from local package
    controller_config = os.path.join(
        pkg_share, 'config', 'controller', 'controller_manager.yaml'
    )

    cartesian_motion_config = os.path.join(
        pkg_share, 'config', 'controller', 'agilex_piper_cartesian_motion_controller.yaml'
    )

    hand_position_config = os.path.join(
        pkg_share, 'config', 'controller', 'inspire_rh56_hand_joint_position_controller.yaml'
    )

    # Foxglove bridge launch file
    foxglove_bridge_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    # Nodes
    nodes: List[Node] = [
        # Controller manager for both arm and hand
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                robot_description,
                controller_config,
            ],
            remappings=[
                ('/controller_manager/robot_description', '/robot_description'),
            ],
        ),
        # Robot state publisher for combined system
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        # Joint state broadcaster (publishes states from both arm and hand)
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        # Arm Cartesian motion controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='cartesian_motion_controller_spawner',
            output='screen',
            arguments=[
                'agilex_piper_cartesian_motion_controller',
                '--controller-manager', '/controller_manager',
                '--param-file', cartesian_motion_config,
            ],
        ),
        # Hand position controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='hand_position_controller_spawner',
            output='screen',
            arguments=[
                'inspire_rh56_hand_joint_position_controller',
                '--controller-manager', '/controller_manager',
                '--param-file', hand_position_config,
            ],
        ),
        # Foxglove bridge for web-based visualization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_bridge_launch)
        ),
    ]

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for integrated Piper arm and Inspire hand system."""
    # Declare arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can2',
        description='CAN interface for arm hardware communication'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for hand communication'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for hand serial communication'
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for testing (true/false)'
    )

    hand_side_arg = DeclareLaunchArgument(
        'hand_side',
        default_value='left',
        description='Which hand to control: left or right'
    )

    return LaunchDescription([
        can_interface_arg,
        serial_port_arg,
        baudrate_arg,
        use_mock_hardware_arg,
        hand_side_arg,
        OpaqueFunction(function=launch_setup)
    ])
