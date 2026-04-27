## Head Control Package
This package is being used to control the 2-DOF head of the Unitree G1.
The head is comprised of two Dynamixel motors (Protocol 2.0):
- **Motor ID 1** — Pitch (tilt down only from neutral)
- **Motor ID 2** — Yaw (left/right)

Connected via U2D2 on `/dev/ttyUSB0` at 57600 baud.

### Prerequisites

1. Follow the instructions on this page :
    - [Dynamixel ROS2 Setup](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/#dynamixel-quick-start-guide-for-ros-2)

2. Always source the robotis_ws before running the nodes in this package.
    ```
    source ~/robotis_ws/install/setup.bash
    ```

### How to run

1. Build the package:
    ```
    cd ~/wbcG1/headControl
    colcon build --packages-select head_control
    source install/setup.bash
    ```

2. Run the node:
    ```
    ros2 run head_control head_control_node
    ```

On startup the head drives to the neutral (forward-facing) position automatically.

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/head/target` | `sensor_msgs/JointState` | Subscribed | Target pitch/yaw in radians |
| `/head/state` | `sensor_msgs/JointState` | Published | Current pitch/yaw in radians at 50 Hz |

### Coordinate Convention

- **Pitch**: negative = tilt down, 0 = neutral. Range: `0` to `-1.2 rad`
- **Yaw**: positive = turn right, negative = turn left. Range: `-1.57` to `+1.57 rad`

### Example Commands

Move to neutral:
```
ros2 topic pub --once /head/target sensor_msgs/msg/JointState "{name: ['pitch', 'yaw'], position: [0.0, 0.0]}"
```

Tilt down:
```
ros2 topic pub --once /head/target sensor_msgs/msg/JointState "{name: ['pitch'], position: [-0.5]}"
```

Turn right:
```
ros2 topic pub --once /head/target sensor_msgs/msg/JointState "{name: ['yaw'], position: [0.5]}"
```

### Hardware Config

| Parameter | Value |
|---|---|
| Device | `/dev/ttyUSB0` |
| Baud rate | 57600 |
| Protocol | 2.0 |
| Operating mode | Extended Position Control (mode 4) |
| Pitch motor ID | 1 |
| Yaw motor ID | 2 |
| Pitch neutral tick | 3736 |
| Yaw neutral tick | 3624 |
