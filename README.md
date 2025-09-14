# piper_arm_inspire_hand_bringup

ROS 2 package that provides launch files, controller configurations, and robot descriptions for the integrated Agilex Piper 6-DOF robotic arm with Inspire RH56 dexterous hand system. This package contains everything needed to bring up and operate the complete arm-hand assembly.

## Features
- Launch files for the integrated arm-hand system
- Controller configurations for both arm and hand controllers
- Complete robot URDF descriptions (left and right hand configurations)
- Foxglove Studio configurations for visualization
- Display-only launch file for visualization without hardware

## Related Packages
- **agilex_piper_ros2_control**: Hardware interface for the Piper arm
- **inspire_rh56_hand_ros2_control**: Hardware interface for the Inspire hand
- **agilex_piper_arm_description**: Visual and kinematic description of the arm
- **inspire_rh56_hand_description**: Visual and kinematic description of the hand

## Package Layout
- `launch/`: Launch files for the integrated system
- `config/controller/`: Controller configurations for arm and hand
- `config/foxglove/`: Foxglove Studio layouts for visualization
- `urdf/`: Complete robot URDF descriptions and flange macro

## Prerequisites
- ROS 2 (Jazzy or newer) with `ros2_control` and `ros2_controllers` ecosystem
- Cartesian controllers: `cartesian_motion_controller`
- Hardware interface packages: `agilex_piper_ros2_control`, `inspire_rh56_hand_ros2_control`
- Description packages: `agilex_piper_arm_description`, `inspire_rh56_hand_description`

## Launch Modes

### Main System Launch
Launch the complete integrated arm-hand system:
```bash
ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py
```

### Display Launch (Visualization Only)
Launch for visualization without hardware:
```bash
ros2 launch piper_arm_inspire_hand_bringup display_foxglove.launch.py
```

## Launch Arguments
All launch files support the following arguments:
- `can_interface`: CAN interface for arm communication (default: "can2")
- `serial_port`: Serial port for hand communication (default: "/dev/ttyUSB0")
- `baudrate`: Serial communication baudrate (default: "115200")
- `use_mock_hardware`: Use mock hardware for testing (default: "false")
- `hand_side`: Hand configuration: "left" or "right" (default: "left")

### Examples
```bash
# Physical system with custom interfaces
ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py \
  can_interface:=can1 serial_port:=/dev/ttyUSB1

# Right hand configuration
ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py hand_side:=right

# Simulation mode (no physical hardware)
ros2 launch piper_arm_inspire_hand_bringup piper_arm_inspire_hand.launch.py use_mock_hardware:=true
```

## Controllers
The system starts with:
- **joint_state_broadcaster**: Publishes joint states for both arm and hand
- **agilex_piper_cartesian_motion_controller**: Cartesian space motion control for the arm
- **inspire_rh56_hand_joint_position_controller**: Direct position control for hand joints

## Robot Descriptions
Complete robot URDF files in `urdf/`:
- `piper_arm_inspire_hand_left.urdf.xacro`: Left hand configuration
- `piper_arm_inspire_hand_right.urdf.xacro`: Right hand configuration
- `flange_macro.xacro`: Custom flange for mounting interface

## Testing
### Manual Commands
**Arm Cartesian Control:**
```bash
ros2 topic pub --once /agilex_piper_cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.3, y: 0.0, z: 0.2},
    orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
  }
}"
```

**Hand Position Control:**
```bash
# Open hand
ros2 topic pub --once /inspire_rh56_hand_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Close hand
ros2 topic pub --once /inspire_rh56_hand_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.3, 0.6, 1.4, 1.4, 1.4, 1.4]}"
```

## Introspection
After launching, you can inspect the system:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

## Foxglove Studio
Configurations available at `config/foxglove/`:
- `arm_hand_ros2_control.json`: Complete system visualization
- `display_robot.json`: Robot model display

Connect Foxglove Studio to `ws://localhost:8765` after launching.

## Hardware Setup
For physical operation:
1. **Arm**: Connect via CAN interface and configure: `sudo ip link set can0 up type can bitrate 1000000`
2. **Hand**: Connect via USB-to-Serial and ensure permissions: `sudo usermod -a -G dialout $USER`

## License and maintainers
Refer to `package.xml` for license and maintainer information.