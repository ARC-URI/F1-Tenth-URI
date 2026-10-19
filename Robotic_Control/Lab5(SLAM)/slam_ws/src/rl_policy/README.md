# rl_policy

Drives an F1TENTH car with the LiDAR policy trained in
`AI/reinforcement learning/arc_rl.py`. The same node runs in the AutoDRIVE
simulator and on the physical car, because it only uses the standard F1TENTH
topics:

| Topic    | Type                                   | Direction | Used for                         |
|----------|----------------------------------------|-----------|----------------------------------|
| `/scan`  | `sensor_msgs/LaserScan`                | in        | LiDAR ranges                     |
| `/odom`  | `nav_msgs/Odometry`                    | in        | forward speed (`twist.linear.x`) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | out       | steering angle + target speed    |

Training exports `f1tenth_policy.npz` (copy it into `models/` and rebuild to make it the default): the network weights plus the contract
they were trained with (ray count and field of view, speed range, steering
limit, control rate). The node reads the contract from the file, so the only
dependency on the car is NumPy.

## Simulator

With the AutoDRIVE bridge running and the simulator connected in Autonomous
driving mode:

```bash
ros2 launch rl_policy sim_policy.launch.py max_speed:=0.6
```

This also starts `autodrive_adapter_node`, which turns the simulator's topics
into `/scan` and `/odom` and executes `/drive`.

## Physical car

Start the car's normal driver stack (LiDAR, VESC, ackermann mux, joystick
teleop), then:

```bash
ros2 run rl_policy policy_node --ros-args -p max_speed:=0.5
```

Check before the first run:

1. **LiDAR orientation.** With the car facing a wall and open space to its
   left, the node's log line (`lidar R/C/L ...`) must show a short centre
   reading and a long left reading. If the LiDAR's zero angle does not point
   forward, set `lidar_yaw_offset` to the scan angle (rad) that does.
2. **Speed sign.** `/odom` must report positive `twist.linear.x` when the car
   rolls forward.
3. **Steering sign.** A positive `steering_angle` on `/drive` must turn the
   wheels left.
4. **Wheels off the ground first**, with a hand on the joystick deadman.

## Parameters

| Parameter          | Default  | Meaning                                                         |
|--------------------|----------|-----------------------------------------------------------------|
| `weights_path`     | packaged | Exported `f1tenth_policy.npz`; defaults to `models/` in this package |
| `max_speed`        | `1.0`    | Speed cap in m/s. Raise it once the car laps cleanly.           |
| `lidar_yaw_offset` | `0.0`    | Scan angle (rad) that points straight ahead                     |
| `control_rate`     | `0.0`    | Upper limit on decisions per second; 0 uses the trained rate    |
| `sensor_timeout`   | `0.5`    | Seconds without a scan or odometry before commanding a stop     |
| `scan_topic`, `odom_topic`, `drive_topic` | `/scan`, `/odom`, `/drive` | Topic names          |
