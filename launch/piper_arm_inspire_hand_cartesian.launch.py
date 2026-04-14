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
Launch file for Agilex Piper Arm with Inspire RH56 Hand integrated system using ROS2 Cartesian controllers.

This launch file starts:
- ros2_control_node: Main controller manager for both arm and hand hardware interfaces
- robot_state_publisher: Publishes TF transforms from combined URDF
- joint_state_broadcaster: Publishes joint states from both arm and hand hardware
- cartesian_motion_controller: Provides ROS2 Cartesian space motion control for the arm
- hand_position_controller: Provides direct position commands for hand joints
- hand_gripper_action_adapter: Converts gripper commands to hand joint positions
- gpio_controller: Provides extended arm features (administrative control, pose feedback, status monitoring)
- foxglove_bridge: WebSocket bridge for Foxglove Studio visualization

Key differences from native control launch:
- Uses ROS2 Cartesian motion controller instead of native hardware control
- Supports mock hardware simulation for testing
- Compatible with standard ROS2 control ecosystem
- Suitable for applications requiring ROS2-based motion planning integration

Parameters:
  can_interface (string, default='can2'):
    CAN interface name for arm hardware communication (e.g., 'can0', 'can1', 'can2').
    Only relevant when use_mock_hardware=false.

  serial_port (string, default='/dev/ttyUSB0'):
    Serial port for hand communication.

  baudrate (string, default='115200'):
    Baudrate for hand serial communication.

  use_mock_hardware (bool, default='false'):
    Enable mock hardware simulation for testing without physical robot.
    Set to 'true' for safe testing and development.

  hand_side (string, default='left'):
    Which hand to control: 'left' or 'right'.

  gripper_mapping (string, default='gripper_joint_mapping_3finger.yaml'):
    Gripper mapping configuration file for hand gripper interface.

Usage:
  # For physical robot with CAN and serial interfaces:
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand_cartesian.launch.py
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand_cartesian.launch.py can_interface:=can1 serial_port:=/dev/ttyUSB1

  # For right hand configuration:
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand_cartesian.launch.py hand_side:=right

  # For SIMULATION/TESTING without physical hardware (RECOMMENDED for testing):
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand_cartesian.launch.py use_mock_hardware:=true

  # Use different gripper configurations:
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand_cartesian.launch.py gripper_mapping:=gripper_joint_mapping_2finger.yaml
  ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand_cartesian.launch.py gripper_mapping:=gripper_joint_mapping_3finger.yaml

  Then connect Foxglove Studio to ws://<foxglove_bridge_ip>:8765

Test ROS2 Cartesian motion commands:
  ros2 topic pub --once /agilex_piper_cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped "
  {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.2, y: 0.0, z: 0.2},
      orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
    }
  }"

  # Monitor current pose and arm status
  ros2 topic echo /agilex_piper_gpio_controller/gpio_states

  # Enable/disable arm
  ros2 topic pub --once /agilex_piper_gpio_controller/commands
    control_msgs/msg/DynamicInterfaceGroupValues
    "{interface_groups: ['arm_admin'], interface_values: [{interface_names: ['enable_arm'], values: [1.0]}]}"

Test hand commands:
  # Direct joint control:
  ros2 topic pub --once /inspire_rh56_hand_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.3, 0.6, 0.0, 0.0, 1.4, 1.4]}"

  # Use standard gripper action interface:
  ros2 action send_goal /hand_gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [0.025], effort: [10.0]}}"

  # Or use simple topic interface:
  ros2 topic pub /hand_gripper_command control_msgs/msg/GripperCommand "{position: 0.03, max_effort: 10.0}"

Or use Foxglove Studio's native publisher panel for interactive control.

Observe the integrated system moving in Foxglove Studio visualization.

NOTE: Use 'use_mock_hardware:=true' for simulation or safe testing without physical hardware!
      For native hardware Cartesian control, use piper_arm_inspire_hand_native_cartesian.launch.py
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
    gripper_mapping_value = LaunchConfiguration('gripper_mapping').perform(context)

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
        pkg_share, 'config', 'controller',
        'controller_manager_mock.yaml' if use_mock_hardware_value.lower() == 'true'
        else 'controller_manager.yaml'
    )

    cartesian_motion_config = os.path.join(
        pkg_share, 'config', 'controller', 'agilex_piper_cartesian_motion_controller.yaml'
    )

    gpio_config = os.path.join(
        pkg_share, 'config', 'controller', 'agilex_piper_gpio_controller.yaml'
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
        # Controller manager for both arm and hand with ROS2 Cartesian control
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                robot_description,
                controller_config,
                cartesian_motion_config,
                gpio_config,
                hand_position_config,
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
            ],
        ),
        # GPIO controller for extended arm features
        Node(
            package='controller_manager',
            executable='spawner',
            name='gpio_controller_spawner',
            output='screen',
            arguments=[
                'agilex_piper_gpio_controller',
                '--controller-manager', '/controller_manager',
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
            ],
        ),
        # Hand gripper action adapter for gripper command interface
        Node(
            package='dexhand_utils',
            executable='hand_gripper_action_adapter',
            name='hand_gripper_action_adapter',
            output='screen',
            parameters=[
                {
                    'action_server_name': 'gripper_cmd',
                    'gripper_command_topic': 'gripper_command',
                    'position_controller_topic': '/inspire_rh56_hand_joint_position_controller/commands',
                    'mapping_config_file': gripper_mapping_value,
                    'execution_duration': 1.0,
                }
            ],
        ),
        # Foxglove bridge for web-based visualization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_bridge_launch)
        ),
    ]

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for integrated Piper arm and Inspire hand system with ROS2 Cartesian control."""
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

    gripper_mapping_arg = DeclareLaunchArgument(
        'gripper_mapping',
        default_value='gripper_joint_mapping_3finger.yaml',
        description='Gripper mapping configuration: gripper_joint_mapping_3finger.yaml or gripper_joint_mapping_2finger.yaml'
    )

    return LaunchDescription([
        can_interface_arg,
        serial_port_arg,
        baudrate_arg,
        use_mock_hardware_arg,
        hand_side_arg,
        gripper_mapping_arg,
        OpaqueFunction(function=launch_setup)
    ])
