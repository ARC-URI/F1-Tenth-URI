# AutoDRIVE + ROS 2 Humble SLAM Bring-Up (Ackermann F1TENTH)

This package wires up **slam_toolbox** against the **AutoDRIVE Simulator's**
F1TENTH vehicle, which is a true Ackermann-steered (car-like, non-holonomic)
digital twin -- not a diff-drive robot faking it. It gives you:

- A live-built 2D occupancy map while you drive (`slam_toolbox`, async mode)
- Ackermann steering control via the standard `ackermann_msgs/AckermannDriveStamped`
  message on `/drive`
- A keyboard teleop node (WASD) for manual driving/mapping
- RViz preloaded with the map, scan, and TF tree

It does **not** include the AutoDRIVE Simulator itself (that's a separate
Unity-based application you download from Tinker-Twins/AutoDRIVE), nor does
it reimplement Ackermann kinematics in the vehicle -- the F1TENTH digital
twin already has real independent front-wheel steering. This package only
adapts its ROS 2 topics into a SLAM-ready pipeline.

---

## 0. Why an "adapter" instead of talking to AutoDRIVE directly?

`slam_toolbox` and most ROS 2 nav tools expect standard message types:
`sensor_msgs/LaserScan`, `nav_msgs/Odometry`, TF transforms, and (for
steering) `ackermann_msgs/AckermannDriveStamped`. AutoDRIVE's ROS 2 bridge
publishes its own topic names and, in some message areas, its own simpler
types (e.g. normalized `Float32` throttle/steering commands). The
`autodrive_adapter_node` in this package is the translation layer so you
never have to touch AutoDRIVE-specific message handling in your own SLAM/
navigation code -- you just publish/subscribe standard ROS 2 messages.

---

## 1. Install ROS 2 Humble + dependencies

```bash
# ROS 2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-ackermann-msgs \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  xterm   # only needed if you use the use_teleop:=true launch convenience
```

## 2. Install AutoDRIVE Simulator + ROS 2 bridge

Follow Tinker-Twins' instructions for your platform (Docker is the simplest
route and matches what the AutoDRIVE F1TENTH Sim-Racing league uses):

- Simulator + Devkit: https://github.com/Tinker-Twins/AutoDRIVE
- F1TENTH-specific ROS 2 packages: https://github.com/Tinker-Twins/AutoDRIVE-F1TENTH
  (see the `ROS 2 Packages` folder)
- Docker image (prebuilt bridge): `autodriveecosystem/autodrive_f1tenth_api`

Typical Docker flow:
```bash
xhost local:root
docker run --name autodrive_f1tenth_api --rm -it \
  --network=host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all \
  autodriveecosystem/autodrive_f1tenth_api:<TAG>

# Inside the container:
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

Launch the standalone AutoDRIVE Simulator executable separately (or inside
the same container setup, per their docs), select the F1TENTH vehicle, and
set it to manual/autonomous mode as needed.

## 3. CONFIRM your topic names (important - do this before building)

AutoDRIVE's exact topic names can vary slightly between releases. Once the
simulator + bridge are running:

```bash
ros2 topic list
ros2 topic info /autodrive/f1tenth_1/lidar -v   # confirm message type too
```

Open `src/autodrive_slam_bringup/config/topics.yaml` and edit the
`autodrive_*_topic` values to match exactly what you see. This is the
**only file** you should need to touch to adapt this package to your
specific AutoDRIVE version. If the LiDAR topic's message type is not
`sensor_msgs/LaserScan` in your installed version, you'll also need to
adjust the subscription type in `scripts/autodrive_adapter_node.py`
(`on_lidar` callback) -- the comment there flags exactly where.

## 4. Build this workspace

```bash
cd autodrive_slam_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 5. Run it

With AutoDRIVE Simulator + bridge already running in another terminal:

```bash
ros2 launch autodrive_slam_bringup slam_bringup.launch.py
```

This starts the adapter node, the static `base_link -> laser` transform,
`slam_toolbox`, and RViz (set `use_rviz:=false` to skip RViz).

In a second terminal, drive the car manually to build the map:

```bash
ros2 launch autodrive_slam_bringup teleop.launch.py
```

Controls: `w`/`s` speed up forward/reverse, `a`/`d` steer left/right,
space bar zeroes speed, `q` zeroes steering, `x` quits.

Drive a loop or two around your environment. You'll see the occupancy map
fill in live in RViz as `slam_toolbox` matches successive LiDAR scans.

## 6. Save your map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

or, using slam_toolbox's own serializer (lets you resume/extend mapping later):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/$USER/my_map'}"
```

## 7. Publish drive commands programmatically

Any node (your own planner, a pure-pursuit controller, Nav2, etc.) can drive
the car by publishing to `/drive`:

```bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {frame_id: 'base_link'}, drive: {speed: 1.0, steering_angle: 0.2}}" -r 10
```

`speed` is in m/s, `steering_angle` is the virtual bicycle-model angle in
radians (positive = left), clamped internally to `max_steering_angle`
(default 0.4189 rad / ~24 deg, the stock F1TENTH servo limit -- edit in
`topics.yaml` if your vehicle differs).

---

## File overview

```
autodrive_slam_bringup/
├── config/
│   ├── topics.yaml              # <-- EDIT THIS to match your AutoDRIVE topic names
│   └── slam_toolbox_params.yaml # SLAM tuning for a fast car-like robot
├── launch/
│   ├── slam_bringup.launch.py   # main entry point
│   └── teleop.launch.py         # standalone keyboard teleop
├── rviz/
│   └── slam_view.rviz           # preloaded Map/Scan/TF/Odometry display
├── scripts/
│   ├── autodrive_adapter_node.py        # AutoDRIVE <-> standard ROS 2 bridge
│   └── ackermann_keyboard_teleop.py     # WASD -> AckermannDriveStamped
├── package.xml
└── CMakeLists.txt
```

## Troubleshooting

- **No map appears in RViz**: check `ros2 topic hz /scan` and `/odom` are
  both publishing. If `/scan` is silent, your `autodrive_lidar_topic` name
  or message type in `topics.yaml` / the adapter node is likely wrong --
  re-check with `ros2 topic list` / `ros2 topic info -v`.
- **Map drifts heavily during turns**: the adapter's odometry is a pure
  bicycle-model integration with no slip model, so it will drift faster than
  a real wheel encoder-based estimate, especially at high speed or sharp
  steering. `slam_toolbox`'s scan matching corrects most of this, but for
  best results drive at moderate speed and close loops periodically. If
  AutoDRIVE exposes wheel encoder topics in your version, you can extend
  `on_drive_cmd`/add new subscriptions to fuse real encoder feedback instead
  of relying purely on the commanded speed.
- **Car doesn't move from `/drive` commands**: confirm `autodrive_throttle_topic`
  and `autodrive_steering_topic` in `topics.yaml` exactly match what the
  AutoDRIVE bridge subscribes to (`ros2 topic info <topic> -v` shows
  subscriber count).
