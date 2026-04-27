# G1 Head Control

**Author:** Jyothiswaroop Kasina

ROS 2 package for the 2-DOF head of the Unitree G1 robot.

<!-- gif of head moving goes here -->

## Packages

| Package | Description |
|---|---|
| [`head_control`](head_control/) | ROS 2 node to drive the two Dynamixel motors (pitch and yaw) via Dynamixel SDK. Subscribes to `/head/target` and publishes current joint state on `/head/state`. |

## Getting Started

After cloning, build the knowledge graph for AI-assisted development:

```bash
/graphify .
```

This generates `graphify-out/` locally (not tracked in git) and enables Claude Code to answer architecture questions about the codebase. The graph rebuilds automatically on every `git commit` from that point on.

## Neck Design

The mechanical neck design is based on the [TWIST2](https://yanjieze.com/TWIST2/) project.
