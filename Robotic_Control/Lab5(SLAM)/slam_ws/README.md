ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/texdrew/my_map'}"# AutoDRIVE F1TENTH SLAM Workspace

A ROS 2 (Humble) workspace that runs live SLAM (`slam_toolbox`) against the
[AutoDRIVE Simulator](https://autodrive-ecosystem.github.io)'s F1TENTH vehicle.

- **`autodrive_f1tenth`** (the "devkit") — the AutoDRIVE-maintained ROS 2
  bridge. Connects to the AutoDRIVE Simulator over a Socket.IO/WebSocket
  TCP connection and exposes its sensors/actuators as ROS 2 topics.
- **`autodrive_slam_bringup`** — an adapter node that normalizes those
  topics into a standard ROS 2 nav stack (`/scan`, `/odom`, TF), plus a
  `slam_toolbox` bring-up, RViz config, and keyboard teleop.

The simulator itself (a large Unity binary) is **not** included in this
repo — download it separately from
[AutoDRIVE's releases](https://github.com/Tinker-Twins/AutoDRIVE).

## 1. Install dependencies

```bash
cd autodrive_ws        # this workspace's root (contains src/)
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/autodrive_devkit/requirements_python_3.10.txt
```

Use `requirements_python_3.8.txt` / `_3.9.txt` instead if your system Python
is older — check with `python3 --version`.

**The pinned `python-socketio`/`python-engineio` versions matter.** The
AutoDRIVE Simulator's Unity client speaks an older Socket.IO/Engine.IO
protocol version. If you `pip install` a newer python-socketio/engineio for
any other project later and it silently upgrades these, the bridge will
fail to handshake with a `Connected!` log line immediately followed by
errors. Re-run the pinned install above to fix it.

## 2. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Get the AutoDRIVE Simulator

Download the build for your platform from AutoDRIVE's releases and unzip it
anywhere convenient. There are two supported layouts:

### Option A — Windows host + WSL2 (ROS 2 runs in WSL, simulator runs on Windows)

This is the setup if you're on Windows: ROS 2/Linux only runs properly
inside WSL2, but the AutoDRIVE Simulator is a native Windows/Unity binary
that runs directly on Windows, talking to the WSL2 VM over a virtual
network link.

1. Install WSL2 + Ubuntu 22.04, then ROS 2 Humble inside it, if you haven't
   already (see [ROS 2 install docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)).
2. Clone/copy this workspace into your WSL2 home directory and follow
   steps 1-2 above **inside WSL**.
3. From inside WSL, find WSL's IP address:
   ```bash
   hostname -I
   ```
4. Launch the *AutoDRIVE Simulator.exe* on Windows. In its connection/bridge
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

## 4. Driving the car

You can drive either via the simulator's own native keyboard controls, or
via ROS using the included teleop:

```bash
ros2 launch autodrive_slam_bringup slam_bringup.launch.py use_teleop:=true
```

Odometry is read directly from AutoDRIVE's ground-truth position/IMU
orientation (see `autodrive_adapter_node.py`), so it tracks correctly
either way — it does not depend on commands being sent over `/drive`.

## 5. Viewing the map

RViz is launched by default (`use_rviz:=true`) and will show the live
LiDAR scan, TF tree, and odometry. **The RViz "Map" display panel itself
is disabled by default** — on some Mesa/virtualized-GPU setups (notably
WSLg) it crashes with a `GLSL link result: active samplers with a
different type refer to the same texture image unit` error. This is a
known RViz2/Ogre rendering bug unrelated to whether SLAM is actually
working. To inspect the actual map, export it directly instead:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This writes `my_map.pgm` (the occupancy grid image) and `my_map.yaml`
(metadata), viewable in any image viewer.

If you want to try the live Map display anyway, re-enable it in
`src/autodrive_slam_bringup/rviz/slam_view.rviz` (set the Map display's
`Enabled`/`Value` to `true`).

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
