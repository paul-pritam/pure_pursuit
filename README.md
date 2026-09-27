# Pure Pursuit Controller

A standalone C++ ROS 2 node implementing the Pure Pursuit path tracking algorithm for differential-drive mobile robots.

The node subscribes to a global path, identifies a dynamic lookahead point, calculates the required geometric curvature, and publishes velocity commands to steer the robot along the trajectory.

## Why Build a Custom Pure Pursuit Node?

Modern navigation frameworks like Nav2 provide comprehensive trajectory tracking plugins, but their internal layers can obscure low-level kinematic issues during initial chassis bring-up.

This standalone node was written to isolate and verify differential-drive path tracking from first geometric principles. Having direct, register-to-velocity visibility makes it straightforward to tune lookahead behavior, calibrate wheel encoders, and diagnose why a vehicle oscillates or cuts corners before integrating complex behavior trees.

## Mathematical Formulation

The controller uses geometric circular arc tracking. Given the robot pose in the odometry frame and the target lookahead waypoint, the algorithm transforms the waypoint into the robot base frame (`base_link`):

```
T_base_to_la = (T_world_to_robot)^(-1) * T_world_to_la
```

The chord distance between the robot origin and the lookahead point is:

```
L_d^2 = x_la^2 + y_la^2
```

The instantaneous curvature kappa of the circular arc connecting the base frame to the lookahead point is:

```
kappa = (2 * y_la) / L_d^2
```

The commanded angular velocity omega is proportional to curvature, while the linear velocity v is maintained at the configured limit:

```
v = max_linear_vel
omega = kappa * max_angular_vel
```

When the distance to the final path waypoint drops below 0.1 meters, the node publishes a zero velocity command and clears the plan.

## Node Architecture

```
                 +-----------------------------+
/astar/path ---> |                             | ---> /cmd_vel
(nav_msgs/Path)  |      pure_pursuit_node      |      (geometry_msgs/Twist)
                 |                             |
/tf -----------> | - 10 Hz Control Loop        | ---> /pure_pursuit/look_ahead_pose
(odom->base)     | - Dynamic Frame Transform   |      (geometry_msgs/PoseStamped)
                 | - Backward Lookahead Search |
                 +-----------------------------+
```

### Path Ingestion & Transform

- **Coordinate Frame Matching**: If the incoming path frame differs from the robot odometry frame, waypoints are transformed into the local frame using `tf2::doTransform`.
- **Lookahead Search**: Scans backwards from the end of the path vector to locate the earliest waypoint whose Euclidean distance from the robot exceeds `look_ahead_dist`.
- **Quality of Service (QoS)**: The path subscriber is configured with `transient_local` durability and `reliable` reliability to reliably ingest latching planner topics.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `look_ahead_dist` | double | `0.5` | Lookahead horizon distance in meters |
| `max_linear_vel` | double | `0.3` | Forward linear velocity command in m/s |
| `max_angular_vel` | double | `1.0` | Angular velocity multiplier in rad/s |

## Topics & Transforms

### Subscribed
- `/astar/path` (`nav_msgs/msg/Path`): Global reference plan.
- `/tf` (`tf2_msgs/msg/TFMessage`): Coordinate transforms for `odom -> base_link`.

### Published
- `/cmd_vel` (`geometry_msgs/msg/Twist`): Velocity commands for the mobile base.
- `/pure_pursuit/look_ahead_pose` (`geometry_msgs/msg/PoseStamped`): Current active lookahead target pose for visualization in RViz.

## Building

Source your ROS 2 installation and compile with `colcon`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select pure_pursuit --symlink-install
```

## Running

Run the node directly with default parameters:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pure_pursuit pure_pursuit_node
```

Override parameters from the command line:

```bash
ros2 run pure_pursuit pure_pursuit_node --ros-args \
  -p look_ahead_dist:=0.6 \
  -p max_linear_vel:=0.35 \
  -p max_angular_vel:=1.2
```

## Tuning Guidelines

- **Under-steering / Corner Cutting**: Reduce `look_ahead_dist` to tighten path tracking around sharp bends.
- **Oscillation / S-Curves**: Increase `look_ahead_dist` if the robot exhibits high-frequency oscillations along straight corridors.
- **Heading Stability**: Adjust `max_angular_vel` to cap rotational aggressiveness during large heading deviations.
- **Transform Overhead**: Transforming global path poses into the local odometry frame in `transform_plan()` prevents frame mismatch failures when switching between map-based and odometry-based navigation.

## License

This project is licensed under the Apache License 2.0.
