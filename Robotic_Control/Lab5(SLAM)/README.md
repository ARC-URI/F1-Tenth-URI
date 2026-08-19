# AutoDRIVE F1TENTH SLAM Workspace

A ROS 2 (Humble) workspace that runs live SLAM (`slam_toolbox`) against the
[AutoDRIVE Simulator](https://autodrive-ecosystem.github.io)'s F1TENTH
vehicle — a true Ackermann-steered (car-like, non-holonomic) digital twin,
not a diff-drive robot faking it.

This workspace is made up of two packages:

- **`autodrive_devkit`** (a.k.a. `autodrive_f1tenth`) — the AutoDRIVE-maintained
  ROS 2 API/bridge. Connects to the AutoDRIVE Simulator over a
  Socket.IO/WebSocket TCP connection and exposes its sensors/actuators as
  ROS 2 topics.
- **`autodrive_slam_bringup`** — an adapter node that normalizes those
  topics into a standard ROS 2 nav stack (`/scan`, `/odom`, TF,
  `ackermann_msgs/AckermannDriveStamped` on `/drive`), plus a
  `slam_toolbox` bring-up, RViz config, and keyboard teleop.

Together they give you:

- A live-built 2D occupancy map while you drive (`slam_toolbox`, async mode)
- Ackermann steering control via the standard
  `ackermann_msgs/AckermannDriveStamped` message on `/drive`
- A keyboard teleop node (WASD) for manual driving/mapping
- RViz preloaded with the scan and TF tree

The simulator itself (a large Unity binary) is **not** included in this
repo — download/run it separately as described below.

### Why an "adapter" instead of talking to AutoDRIVE directly?

`slam_toolbox` and most ROS 2 nav tools expect standard message types:
`sensor_msgs/LaserScan`, `nav_msgs/Odometry`, TF transforms, and (for
steering) `ackermann_msgs/AckermannDriveStamped`. AutoDRIVE's ROS 2 bridge
publishes its own topic names and, in some message areas, its own simpler
types (e.g. normalized `Float32` throttle/steering commands). The
`autodrive_adapter_node` in `autodrive_slam_bringup` is the translation
layer so you never have to touch AutoDRIVE-specific message handling in
your own SLAM/navigation code — you just publish/subscribe standard ROS 2
messages.

Odometry is read directly from AutoDRIVE's ground-truth position/IMU
orientation (see `autodrive_adapter_node.py`), so it tracks correctly
regardless of how the car is driven — it does not depend on commands being
sent over `/drive`.

> **Note:** the adapter's odometry approach determines how well the map
> holds up during aggressive driving. If your build of the adapter node
> instead integrates commanded speed/steering (a pure bicycle-model
> estimate with no slip model), expect faster drift on sharp turns or at
> high speed — check the `on_odom`/`on_drive_cmd` callbacks in
> `autodrive_adapter_node.py` to confirm which source your version uses,
> and drive at a moderate pace / close loops periodically if so.

---

## 1. Install dependencies

```bash
cd slam_ws        # this workspace's root (contains src/)
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/autodrive_devkit/requirements_python_3.10.txt
```

Use `requirements_python_3.8.txt` / `_3.9.txt` instead if your system
Python is older — check with `python3 --version`.

Also give the devkit's scripts executable permissions once:

```bash
cd src/autodrive_devkit
sudo chmod +x *.py
cd -
```

If you're bringing up `autodrive_slam_bringup`, also install its ROS 2
system dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-ackermann-msgs \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  xterm   # only needed if you use the use_teleop:=true launch convenience
```

**The pinned `python-socketio`/`python-engineio` versions matter.** The
AutoDRIVE Simulator's Unity client speaks an older Socket.IO/Engine.IO
protocol version. If you `pip install` a newer python-socketio/engineio for
any other project later and it silently upgrades these, the bridge will
fail to handshake with a `Connected!` log line immediately followed by
errors. Re-run the pinned install above to fix it.

<details>
<summary>Exact pinned versions (tested per Python version)</summary>

Websocket-related dependencies for the communication bridge between the
AutoDRIVE Simulator and the devkit (version-sensitive):

| Package            | Python 3.8 | Python 3.9 | Python 3.10 |
|---------------------|------------|------------|-------------|
| eventlet            | 0.33.3     | 0.33.3     | 0.33.3      |
| Flask               | 1.1.1      | 1.1.1      | 1.1.1       |
| Flask-SocketIO      | 4.1.0      | 4.1.0      | 4.1.0       |
| python-socketio     | 4.2.0      | 4.2.0      | 4.2.0       |
| python-engineio     | 3.13.0     | 3.13.0     | 3.13.0      |
| greenlet            | 1.0.0      | 1.0.0      | 1.1.0       |
| gevent              | 21.1.2     | 21.1.2     | 21.12.0     |
| gevent-websocket    | 0.10.1     | 0.10.1     | 0.10.1      |
| Jinja2              | 3.0.3      | 3.0.3      | 3.0.3       |
| itsdangerous        | 2.0.1      | 2.0.1      | 2.0.1       |
| werkzeug            | 2.0.3      | 2.0.3      | 2.0.3       |

Generic dependencies for data processing and visualization (usually any
recent version works):

| Package               | Tested Version |
|------------------------|----------------|
| numpy                  | 1.13.3         |
| pillow                 | 5.1.0          |
| opencv-contrib-python  | 4.5.1.48       |

These are exactly what's pinned in `requirements_python_3.{8,9,10}.txt` —
install with the matching file for your Python version rather than by
hand.

</details>

## 2. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Get and run the AutoDRIVE Simulator

Download the build for your platform from AutoDRIVE's releases and unzip
it anywhere convenient. There are three supported setups:

### Option A — Windows host + WSL2 (ROS 2 runs in WSL, simulator runs on Windows)

This is the setup if you're on Windows: ROS 2/Linux only runs properly
inside WSL2, but the AutoDRIVE Simulator is a native Windows/Unity binary
that runs directly on Windows, talking to the WSL2 VM over a virtual
network link.

1. Install WSL2 + Ubuntu 22.04, then ROS 2 Humble inside it, if you
   haven't already (see
   [ROS 2 install docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)).
2. Clone/copy this workspace into your WSL2 home directory and follow
   steps 1-2 above **inside WSL**.
3. From inside WSL, find WSL's IP address:
   ```bash
   hostname -I
   ```
4. Launch *AutoDRIVE Simulator.exe* on Windows. In its connection/bridge
   settings, enter the WSL IP from step 3 and port `4567`, then connect.
   (The simulator is the Socket.IO *client*; the bridge running in WSL is
   the *server* listening on port 4567 — this direction, not the reverse.)
5. In WSL, launch the bridge, then the SLAM stack:
   ```bash
   ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
   # in a second terminal:
   ros2 launch autodrive_slam_bringup slam_bringup.launch.py
   ```

**Note:** WSL2's IP address can change across reboots — re-run
`hostname -I` and reconnect the simulator if the bridge won't connect.

### Option B — Native Linux (ROS 2 and simulator both on the same machine)

1. Install ROS 2 Humble on Ubuntu 22.04 (see link above), clone this
   workspace, and follow steps 1-2.
2. Download and run the Linux build of the AutoDRIVE Simulator. In its
   bridge settings, use `127.0.0.1` (loopback — both processes are on the
   same machine) and port `4567`, then connect.
3. Launch the bridge, then the SLAM stack, exactly as in step 5 above.

### Option C — Docker (prebuilt bridge)

Matches what the AutoDRIVE F1TENTH Sim-Racing league uses. See Tinker-Twins'
instructions for full details:

- Simulator + Devkit: https://github.com/Tinker-Twins/AutoDRIVE
- F1TENTH-specific ROS 2 packages: https://github.com/Tinker-Twins/AutoDRIVE-F1TENTH
  (see the `ROS 2 Packages` folder)
- Docker image: `autodriveecosystem/autodrive_f1tenth_api`

Typical flow:

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

## 4. Confirm your topic names (do this before relying on SLAM output)

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
(`on_lidar` callback) — the comment there flags exactly where.

`max_steering_angle` also lives in `topics.yaml` (default `0.4189` rad /
~24°, the stock F1TENTH servo limit) — edit it if your vehicle differs.

## 5. Bringup

- **Headless mode:**
  ```bash
  ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
  ```
  **[OR]**
- **RViz mode:**
  ```bash
  ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
  ```

Then bring up the SLAM stack:

```bash
ros2 launch autodrive_slam_bringup slam_bringup.launch.py
```

This starts the adapter node, the static `base_link -> laser` transform,
`slam_toolbox`, and RViz (`use_rviz:=true` by default; set
`use_rviz:=false` to skip it).

## 6. Driving the car

You can drive either via the simulator's own native keyboard controls, via
the devkit's teleop, or via the SLAM package's Ackermann teleop:

```bash
# devkit teleop
ros2 run autodrive_f1tenth teleop_keyboard

# or, as part of slam_bringup:
ros2 launch autodrive_slam_bringup slam_bringup.launch.py use_teleop:=true
# equivalently, standalone in a second terminal:
ros2 launch autodrive_slam_bringup teleop.launch.py
```

WASD controls for the Ackermann teleop: `w`/`s` speed up forward/reverse,
`a`/`d` steer left/right, space bar zeroes speed, `q` zeroes steering, `x`
quits.

Drive a loop or two around your environment — you'll see the occupancy map
fill in live as `slam_toolbox` matches successive LiDAR scans.

### Publishing drive commands programmatically

Any node (your own planner, a pure-pursuit controller, Nav2, etc.) can
drive the car by publishing to `/drive`:

```bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {frame_id: 'base_link'}, drive: {speed: 1.0, steering_angle: 0.2}}" -r 10
```

`speed` is in m/s, `steering_angle` is the virtual bicycle-model angle in
radians (positive = left), clamped internally to `max_steering_angle`.

## 7. Viewing and saving the map

RViz is launched by default and will show the live LiDAR scan, TF tree,
and odometry. **The RViz "Map" display panel itself is disabled by
default** — on some Mesa/virtualized-GPU setups (notably WSLg) it crashes
with a `GLSL link result: active samplers with a different type refer to
the same texture image unit` error. This is a known RViz2/Ogre rendering
bug unrelated to whether SLAM is actually working. To inspect the actual
map, export it directly instead:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This writes `my_map.pgm` (the occupancy grid image) and `my_map.yaml`
(metadata), viewable in any image viewer.

Alternatively, use `slam_toolbox`'s own serializer, which lets you
resume/extend mapping later:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/$USER/my_map'}"
```

If you want to try the live Map display anyway, re-enable it in
`src/autodrive_slam_bringup/rviz/slam_view.rviz` (set the Map display's
`Enabled`/`Value` to `true`).

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

- **`Connected!` then immediate errors / no data flows**: a Socket.IO
  protocol version mismatch — re-run the pinned `pip install` from step 1.
- **QoS "incompatible policy: RELIABILITY" warnings**: harmless if you see
  them once at startup; the relevant topics are published as Reliable.
- **`Message Filter dropping message ... queue is full` flooding the
  terminal, only in lines tagged `[rviz2-...]`**: this is RViz's own
  display-side TF buffer, not `slam_toolbox`'s. Check
  `~/.ros/log/async_slam_toolbox_node_*/...log` directly to see whether
  `slam_toolbox` itself is dropping anything — that's the signal that
  actually matters for whether the map is building.
- **Map never grows / car appears frozen at the origin**: make sure only
  *one* copy of `slam_bringup.launch.py` is running at a time. Running it
  in two terminals simultaneously creates duplicate `/scan`/`/tf`
  publishers that fight each other and will make the map look stuck or
  corrupted. Check with `ros2 node list` — you should see exactly one
  `autodrive_adapter_node` and one `slam_toolbox`.
- **No map appears in RViz**: check `ros2 topic hz /scan` and `/odom` are
  both publishing. If `/scan` is silent, your `autodrive_lidar_topic` name
  or message type in `topics.yaml` / the adapter node is likely wrong —
  re-check with `ros2 topic list` / `ros2 topic info -v`.
- **Map drifts heavily during turns**: see the odometry note above — if
  your adapter build integrates commanded speed/steering rather than
  ground truth, drift will be faster on sharp turns or at high speed.
  `slam_toolbox`'s scan matching corrects most of this, but for best
  results drive at moderate speed and close loops periodically. If
  AutoDRIVE exposes wheel encoder topics in your version, you can extend
  `on_drive_cmd`/add new subscriptions to fuse real encoder feedback
  instead.
- **Car doesn't move from `/drive` commands**: confirm
  `autodrive_throttle_topic` and `autodrive_steering_topic` in
  `topics.yaml` exactly match what the AutoDRIVE bridge subscribes to
  (`ros2 topic info <topic> -v` shows subscriber count).

---

## Appendix: cloning the devkit standalone

If you need the devkit (`autodrive_devkit`, ROS 2 API for the F1TENTH
vehicle) outside of this workspace, it comes from:

```bash
git clone https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing.git
```

— see [AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing)
and [Tinker-Twins/AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE) for
upstream releases and documentation. In this workspace it already lives at
`src/autodrive_devkit`, so you don't need to clone it separately — just
follow "Install dependencies" above.
