# My Robot MoveIt Workspace

ROS 2 workspace cho robot arm 6-DOF với MoveIt 2 motion planning, ros2_control và Dynamixel hardware interface.

## Tổng quan

Project này bao gồm một robot arm 6 bậc tự do với gripper, được tích hợp đầy đủ với:
- **MoveIt 2** - Motion planning và kinematics
- **ros2_control** - Hardware abstraction layer
- **Dynamixel SDK** - Giao tiếp với servo XL330

## Cấu trúc Package

```
src/
├── my_robot_description/     # URDF/Xacro mô tả robot
├── my_robot_bringup/         # Launch files và config
├── my_robot_moveit_config/   # MoveIt configuration
├── my_robot_hardware/        # Hardware interface cho Dynamixel
├── my_robot_commander_cpp/   # C++ commander node
├── my_robot_commander_py/    # Python commander node
└── my_robot_interfaces/      # Custom ROS messages
```

## Robot Description

### Arm (6-DOF)
| Joint   | Type      | Axis | Limits (rad)       |
|---------|-----------|------|--------------------|
| joint1  | revolute  | Z    | -3.14 → 3.14       |
| joint2  | revolute  | Y    | 0 → 2.5            |
| joint3  | revolute  | Y    | 0 → 2.5            |
| joint4  | revolute  | Z    | -3.14 → 3.14       |
| joint5  | revolute  | Y    | -1.57 → 1.57       |
| joint6  | continuous| Z    | unlimited          |

### Gripper
- 2 ngón tay prismatic với mimic joint
- Khoảng mở: 0 → 0.06m

## Yêu cầu

- ROS 2 (Humble/Iron/Jazzy)
- MoveIt 2
- ros2_control
- Dynamixel SDK (cho real hardware)

## Cài đặt

```bash
# Clone repository
cd ~/my_workspace
git clone git@github.com:dangtruongthinh/moveit_ws.git
cd moveit_ws

# Cài đặt dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

## Sử dụng

### 1. Chạy với Mock Hardware (Simulation)

```bash
ros2 launch my_robot_bringup my_robot.launch.xml use_mock_component:=true
```

### 2. Chạy với Real Hardware (Dynamixel)

```bash
ros2 launch my_robot_bringup my_robot.launch.xml use_mock_component:=false
```

### 3. Chạy MoveIt Demo

```bash
ros2 launch my_robot_moveit_config demo.launch.py
```

## Commander Interface

### Topics

| Topic              | Message Type                        | Mô tả                    |
|--------------------|-------------------------------------|--------------------------|
| `/open_gripper`    | `example_interfaces/msg/Bool`       | true=mở, false=đóng      |
| `/joint_command`   | `example_interfaces/msg/Float64MultiArray` | 6 joint values (rad) |
| `/pose_command`    | `my_robot_interfaces/msg/PoseCommand` | Target pose (x,y,z,rpy) |

### Ví dụ sử dụng

```bash
# Điều khiển gripper
ros2 topic pub /open_gripper example_interfaces/msg/Bool "{data: true}"

# Di chuyển theo joint values
ros2 topic pub /joint_command example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.5, 0.5, 0.0, 0.0, 0.0]}"

# Di chuyển đến pose
ros2 topic pub /pose_command my_robot_interfaces/msg/PoseCommand \
  "{x: 0.3, y: 0.0, z: 0.5, roll: 0.0, pitch: 1.57, yaw: 0.0, cartesian_path: false}"
```

## Hardware Interface

Robot hỗ trợ 2 mode:

1. **Mock Component** (`use_mock_component:=true`)
   - Sử dụng `mock_components/GenericSystem`
   - Không cần hardware thực
   - Phù hợp cho development và testing

2. **Real Hardware** (`use_mock_component:=false`)
   - Sử dụng `arm_hardware/ArmHardwareInterface`
   - Kết nối với Dynamixel XL330 qua serial
   - Port mặc định: `/dev/ttyACM0`

### Cấu hình Motor ID

```xml
<param name="joint1_motor_id">10</param>
<param name="joint2_motor_id">20</param>
<param name="dynamixel_port">/dev/ttyACM0</param>
```

## Controllers

Cấu hình trong `my_robot_bringup/config/ros2_controllers.yaml`:

- `joint_state_broadcaster` - Broadcast joint states
- `arm_controller` - JointTrajectoryController cho arm
- `gripper_controller` - GripperActionController cho gripper

## Custom Messages

### PoseCommand.msg

```
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
bool cartesian_path
```

## Maintainer

- **Thinh** - dtruongthinh2409@gmail.com

## License

TODO: Add license
