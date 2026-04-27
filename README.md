# G1 Head Control

**Author:** Jyothiswaroop Kasina

ROS 2 package for the 2-DOF head of the Unitree G1 robot.

<!-- gif of head moving goes here -->

## Packages

| Package | Description |
|---|---|
| [`head_control`](head_control/) | ROS 2 node to drive the two Dynamixel motors (pitch and yaw) via Dynamixel SDK. Subscribes to `/head/target` and publishes current joint state on `/head/state`. |

## Neck Design

The mechanical neck design is based on the [TWIST2](https://yanjieze.com/TWIST2/) project.
