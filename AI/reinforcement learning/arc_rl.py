# -*- coding: utf-8 -*-
"""ARC_RL: train a LiDAR driving policy for the F1TENTH car in CarRacing-v3 (Box2D).

CarRacing's car is very close to a 10x scale F1TENTH car (wheelbase 3.24 units
vs 0.33 m, track 2.2 units vs 0.22 m, 0.4 rad steering lock), so this script
treats 10 Box2D units as 1 metre and exposes the same interface the real car has:

    observation: LiDAR ranges + forward speed + previous action
    action:      steering angle + target speed   (an AckermannDrive command)

The trained policy is exported as plain NumPy weights together with the
observation/action contract, which the `rl_policy` ROS 2 node reads. The same
node drives the AutoDRIVE simulator and the physical car.

Run as a script (`python arc_rl.py`) or paste into one Colab cell after:
    pip install "stable-baselines3[extra]" "gymnasium[box2d]"
Set the TIMESTEPS environment variable to change the training length, and
INIT_MODEL to a saved model (.zip) to continue training it instead of starting fresh.
"""

import importlib.util
import json
import math
import os

import gymnasium as gym
import matplotlib
import numpy as np
from gymnasium.envs.box2d import car_racing
from gymnasium.envs.box2d.car_dynamics import Car

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

try:
    from IPython import get_ipython
    IN_NOTEBOOK = get_ipython() is not None
except ImportError:
    IN_NOTEBOOK = False
if not IN_NOTEBOOK:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ============================================================
# Contract shared with the car (exported with the weights; do not edit on the car)
# ============================================================
UNITS_PER_METER = 10.0        # Box2D units per metre

LIDAR_NUM_RAYS = 61
LIDAR_FOV = np.pi             # 180 degrees, centred straight ahead
LIDAR_MAX_RANGE = 5.0         # metres; longer readings are clipped to this
LIDAR_OFFSET = 0.10           # metres the LiDAR sits ahead of the wheelbase midpoint
LIDAR_ANGLES = np.linspace(-LIDAR_FOV / 2, LIDAR_FOV / 2, LIDAR_NUM_RAYS)  # right to left

MAX_STEER = 0.4               # rad; action[0] = +1 is full left
MIN_SPEED = 0.5               # m/s; action[1] = -1
MAX_SPEED = 3.0               # m/s; action[1] = +1
SPEED_NORM = 4.0              # m/s; measured speed is divided by this

CONTROL_STEPS = 5             # physics steps (50 Hz) per decision -> 10 Hz control
OBS_DIM = LIDAR_NUM_RAYS + 3

CONTRACT = {
    "lidar_num_rays": LIDAR_NUM_RAYS,
    "lidar_fov": float(LIDAR_FOV),
    "lidar_max_range": LIDAR_MAX_RANGE,
    "max_steer": MAX_STEER,
    "min_speed": MIN_SPEED,
    "max_speed": MAX_SPEED,
    "speed_norm": SPEED_NORM,
    "control_rate": car_racing.FPS / CONTROL_STEPS,
}

# ============================================================
# Training-only settings
# ============================================================
TRACK_HALF_WIDTH = 40.0 / 6.0 / UNITS_PER_METER   # CarRacing's default, 0.67 m (1.33 m wide track)
TRACK_HALF_WIDTH_RANGE = (0.4, 1.0)               # metres, randomized (0.8-2.0 m wide)
# Half of the randomized tracks change width along the lap, like the competition
# track: mostly ~6 ft (1.83 m) wide with ~3 ft (0.91 m) sections.
WIDE_HALF_WIDTH_RANGE = (0.8, 1.0)
NARROW_HALF_WIDTH_RANGE = (0.4, 0.55)
NARROW_FRACTION = 0.35        # share of sections that are narrow
SECTION_TILES = (15, 60)      # section length in track tiles (0.35 m each)
WALL_MARGIN = 0.12            # metres from the track edge at which the car body touches the "wall"
OFF_TRACK_PENALTY = 30.0
STEER_RATE_PENALTY = 0.2      # per unit change of action[0] between decisions
PROXIMITY_RANGE = 0.3         # metres; LiDAR readings closer than this are penalized
PROXIMITY_PENALTY = 0.5       # per decision when the closest reading reaches zero
GAS_GAIN = 0.5                # speed controller: gas per m/s of speed error
BRAKE_GAIN = 0.5
BRAKE_MAX = 0.8
START_HEADING_RANGE = 0.5     # rad; randomized starts point up to this far off the track direction


def car_heading(car):
    """World heading of the car in radians. The hull's forward axis is local +y."""
    return car.hull.angle + math.pi / 2


def forward_speed(car):
    """Forward speed in m/s."""
    heading = car_heading(car)
    vel = car.hull.linearVelocity
    return (vel[0] * math.cos(heading) + vel[1] * math.sin(heading)) / UNITS_PER_METER


def width_profile(n_tiles, rng):
    """Half-width (m) at each track tile: wide and narrow sections with smooth transitions."""
    widths = np.empty(n_tiles)
    i = 0
    while i < n_tiles:
        narrow = rng.random() < NARROW_FRACTION
        length = int(rng.integers(*SECTION_TILES))
        widths[i:i + length] = rng.uniform(*(NARROW_HALF_WIDTH_RANGE if narrow else WIDE_HALF_WIDTH_RANGE))
        i += length
    # Blend the steps over ~2 m, wrapping around the closed lap
    kernel = np.ones(7) / 7
    return np.convolve(np.concatenate([widths[-3:], widths, widths[:3]]), kernel, mode="valid")


def compute_track_boundaries(track, half_widths):
    """Left/right track edges as (N, 2, 2) arrays of segments, in Box2D units.

    Args:
        track: list of (alpha, beta, x, y) tuples from env.unwrapped.track
        half_widths: half the track width at each tile, in Box2D units
    """
    n = len(track)
    left_points = np.zeros((n, 2))
    right_points = np.zeros((n, 2))

    for i, (alpha, beta, cx, cy) in enumerate(track):
        # CarRacing's direction of travel is (-sin(beta), cos(beta)), so
        # (cos(beta), sin(beta)) already points across the track, to the right.
        dx = half_widths[i] * math.cos(beta)
        dy = half_widths[i] * math.sin(beta)
        left_points[i] = [cx - dx, cy - dy]
        right_points[i] = [cx + dx, cy + dy]

    # Build segments: connect consecutive points, wrapping around for closed loop
    left_segs = np.stack([left_points, np.roll(left_points, -1, axis=0)], axis=1)
    right_segs = np.stack([right_points, np.roll(right_points, -1, axis=0)], axis=1)

    return left_segs, right_segs


def cast_lidar_rays(origin, heading, segs, max_range):
    """Distance from `origin` to the nearest segment along each LiDAR ray.

    Everything is in Box2D units. Rays that hit nothing return `max_range`.
    """
    seg_centers = segs.mean(axis=1)
    nearby = segs[np.linalg.norm(seg_centers - origin, axis=1) < max_range + 20.0]
    if len(nearby) == 0:
        return np.full(LIDAR_NUM_RAYS, max_range)

    world_angles = heading + LIDAR_ANGLES
    ray_dirs = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)  # (R, 2)

    A = nearby[:, 0]
    AB = nearby[:, 1] - A
    AP = A - origin

    # Ray/segment intersection for every (ray, segment) pair -> (R, M)
    D_cross_AB = ray_dirs[:, 0, None] * AB[None, :, 1] - ray_dirs[:, 1, None] * AB[None, :, 0]
    AP_cross_AB = AP[:, 0] * AB[:, 1] - AP[:, 1] * AB[:, 0]
    AP_cross_D = AP[None, :, 0] * ray_dirs[:, 1, None] - AP[None, :, 1] * ray_dirs[:, 0, None]

    parallel = np.abs(D_cross_AB) < 1e-10
    D_cross_AB_safe = np.where(parallel, 1.0, D_cross_AB)
    t = AP_cross_AB[None, :] / D_cross_AB_safe   # distance along ray
    s = AP_cross_D / D_cross_AB_safe             # position along segment [0, 1]

    valid = (~parallel) & (t > 1e-6) & (s >= 0) & (s <= 1) & (t <= max_range)
    return np.where(valid, t, max_range).min(axis=1)


def distance_to_polyline(point, points):
    """Shortest distance from `point` to the closed polyline through `points`, and the nearest segment's index."""
    a = points
    ab = np.roll(points, -1, axis=0) - a
    u = np.clip(((point - a) * ab).sum(axis=1) / np.maximum((ab * ab).sum(axis=1), 1e-9), 0.0, 1.0)
    dist = np.linalg.norm(a + u[:, None] * ab - point, axis=1)
    nearest = int(dist.argmin())
    return dist[nearest], nearest


# ============================================================
# CarRacing with the F1TENTH interface
# ============================================================
class F1TenthCarRacing(gym.Wrapper):
    """CarRacing driven like an F1TENTH car.

    Observation (OBS_DIM floats in [-1, 1]):
        [0:LIDAR_NUM_RAYS]  LiDAR ranges, right to left, metres / LIDAR_MAX_RANGE
        [-3]                forward speed, m/s / SPEED_NORM
        [-2], [-1]          the previous action
    Action (2 floats in [-1, 1]):
        [0] steering angle / MAX_STEER, positive = left
        [1] target speed, mapped linearly onto [MIN_SPEED, MAX_SPEED]

    With randomize=True each episode draws its own track width, starting pose,
    sensor noise, actuation delay, control period and speed calibration, so the
    policy does not depend on any one of them. Half of those episodes are also
    mirrored left-to-right, because CarRacing tracks only ever run
    counter-clockwise.
    """

    def __init__(self, env, randomize=True):
        super().__init__(env)
        self.randomize = randomize
        self.rng = np.random.default_rng()
        self.observation_space = gym.spaces.Box(-1.0, 1.0, (OBS_DIM,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        if env.unwrapped.render_mode is None:
            # CarRacing draws its 96x96 image every step even when nothing
            # uses it, which makes it ~80x slower.
            env.unwrapped._render = lambda mode: None

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        rng = self.rng

        if self.randomize:
            varying = rng.random() < 0.5
            half_width = WIDE_HALF_WIDTH_RANGE[1] if varying else rng.uniform(*TRACK_HALF_WIDTH_RANGE)
            self.lidar_noise = rng.uniform(0.0, 0.03)       # metres, std
            self.lidar_dropout = rng.uniform(0.0, 0.05)     # fraction of rays with no return
            self.speed_noise = rng.uniform(0.0, 0.15)       # m/s, std
            self.latency = int(rng.integers(0, 4))          # physics steps before a new command acts
            self.control_steps = int(rng.integers(CONTROL_STEPS - 1, CONTROL_STEPS + 4))  # 6-12.5 Hz
            self.speed_gain = rng.uniform(0.85, 1.15)       # actual speed / commanded speed
            self.speed_cap = rng.uniform(1.0, MAX_SPEED) if rng.random() < 0.5 else MAX_SPEED
            self.mirror = rng.random() < 0.5
        else:
            varying = False
            half_width = TRACK_HALF_WIDTH
            self.lidar_noise = self.lidar_dropout = self.speed_noise = 0.0
            self.latency = 1
            self.control_steps = CONTROL_STEPS
            self.speed_gain = 1.0
            self.speed_cap = MAX_SPEED
            self.mirror = False

        # CarRacing reads this module constant while it builds the road surface.
        # The walls the LiDAR sees and the car must stay inside are ours.
        car_racing.TRACK_WIDTH = half_width * UNITS_PER_METER
        _, info = self.env.reset(seed=seed, options=options)

        track = self.unwrapped.track
        self.half_widths = width_profile(len(track), rng) if varying else np.full(len(track), half_width)
        self.left_segs, self.right_segs = compute_track_boundaries(track, self.half_widths * UNITS_PER_METER)
        self.segs = np.concatenate([self.left_segs, self.right_segs], axis=0)
        self.centerline = np.array([[x, y] for _, _, x, y in track])
        if self.randomize:
            self._randomize_start(track[0])

        self.prev_action = np.zeros(2)
        self.applied = (0.0, 0.0)   # steering angle (rad), target speed (m/s) currently acting
        return self._observe(), info

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        steer = (-action[0] if self.mirror else action[0]) * MAX_STEER
        speed = MIN_SPEED + (action[1] + 1.0) / 2.0 * (MAX_SPEED - MIN_SPEED)
        car = self.unwrapped.car

        reward = -STEER_RATE_PENALTY * abs(action[0] - self.prev_action[0])
        terminated = truncated = False
        info = {}
        for i in range(self.control_steps):
            if i == self.latency:
                self.applied = (steer, min(speed, self.speed_cap) * self.speed_gain)
            steer_now, speed_now = self.applied

            error = speed_now - forward_speed(car)
            gas = float(np.clip(GAS_GAIN * error, 0.0, 1.0))
            brake = float(np.clip(-BRAKE_GAIN * error, 0.0, BRAKE_MAX))
            # CarRacing's steering input is the negated wheel angle in radians
            _, r, terminated, truncated, info = self.env.step(
                np.array([-steer_now, gas, brake], dtype=np.float32))
            reward += r
            if terminated or truncated:
                break

        if self._off_track():
            reward -= OFF_TRACK_PENALTY
            terminated = True
        closest = self.lidar_ranges().min()
        reward -= PROXIMITY_PENALTY * max(0.0, 1.0 - closest / PROXIMITY_RANGE)

        self.prev_action = action
        return self._observe(), float(reward), terminated, truncated, info

    def _randomize_start(self, tile):
        """Replace the car with one placed off-centre and not quite facing along the track."""
        _, beta, x, y = tile
        max_offset = (self.half_widths[0] - WALL_MARGIN - 0.2) * UNITS_PER_METER
        offset = self.rng.uniform(-max_offset, max_offset)
        angle = beta + self.rng.uniform(-START_HEADING_RANGE, START_HEADING_RANGE)
        env = self.unwrapped
        env.car.destroy()
        env.car = Car(env.world, angle, x + offset * math.cos(beta), y + offset * math.sin(beta))

    def _off_track(self):
        car = self.unwrapped.car
        pos = np.array([car.hull.position[0], car.hull.position[1]])
        dist, tile = distance_to_polyline(pos, self.centerline)
        return dist > (self.half_widths[tile] - WALL_MARGIN) * UNITS_PER_METER

    def lidar_pose(self):
        """LiDAR position (Box2D units) and heading (rad)."""
        car = self.unwrapped.car
        heading = car_heading(car)
        offset = LIDAR_OFFSET * UNITS_PER_METER
        origin = np.array([car.hull.position[0] + offset * math.cos(heading),
                           car.hull.position[1] + offset * math.sin(heading)])
        return origin, heading

    def lidar_ranges(self):
        """Noise-free LiDAR ranges in metres."""
        origin, heading = self.lidar_pose()
        max_range = LIDAR_MAX_RANGE * UNITS_PER_METER
        return cast_lidar_rays(origin, heading, self.segs, max_range) / UNITS_PER_METER

    def _observe(self):
        ranges = self.lidar_ranges()
        speed = forward_speed(self.unwrapped.car)
        if self.randomize:
            ranges = ranges + self.rng.normal(0.0, self.lidar_noise, ranges.shape)
            ranges[self.rng.random(ranges.shape) < self.lidar_dropout] = LIDAR_MAX_RANGE
            speed += self.rng.normal(0.0, self.speed_noise)

        if self.mirror:
            ranges = ranges[::-1]

        lidar = np.clip(ranges, 0.0, LIDAR_MAX_RANGE) / LIDAR_MAX_RANGE
        speed = np.clip(speed / SPEED_NORM, -1.0, 1.0)
        return np.concatenate([lidar, [speed], self.prev_action]).astype(np.float32)


ENV_ID = "CarRacing-v3"


def make_car_env(randomize=True, render_mode=None, max_episode_steps=2000):
    env = gym.make(ENV_ID, continuous=True, render_mode=render_mode,
                   max_episode_steps=max_episode_steps, disable_env_checker=True)
    return F1TenthCarRacing(env, randomize=randomize)


def make_env(randomize=True):
    """Factory for the vectorized training/evaluation environments."""
    return lambda: Monitor(make_car_env(randomize=randomize))


# ============================================================
# Export for the car: plain NumPy weights + the contract
# ============================================================
def export_policy(model, path):
    """Save the actor MLP and CONTRACT to `path` (.npz). The car needs only NumPy to run it."""
    policy = model.policy
    layers = [m for m in policy.mlp_extractor.policy_net if hasattr(m, "weight")] + [policy.action_net]
    arrays = {}
    for i, layer in enumerate(layers):
        arrays[f"w{i}"] = layer.weight.detach().cpu().numpy()
        arrays[f"b{i}"] = layer.bias.detach().cpu().numpy()
    np.savez(path, contract=json.dumps(CONTRACT), **arrays)


def run_exported_policy(weights, obs):
    """Deterministic action from exported weights; must match model.predict(deterministic=True)."""
    n_layers = sum(1 for k in weights.files if k.startswith("w"))
    x = obs
    for i in range(n_layers):
        x = weights[f"w{i}"] @ x + weights[f"b{i}"]
        if i < n_layers - 1:
            x = np.tanh(x)
    return np.clip(x, -1.0, 1.0)


# ============================================================
# Plots and video
# ============================================================
def plot_lidar_check(path="lidar_check.png"):
    """Top-down picture: rays should fan out ahead of the car and stop at the track edges."""
    env = make_car_env(randomize=False)
    env.reset(seed=0)
    origin, heading = env.lidar_pose()
    dists = env.lidar_ranges() * UNITS_PER_METER

    plt.figure(figsize=(7, 7))
    for segs, color in [(env.left_segs, "tab:blue"), (env.right_segs, "tab:orange")]:
        plt.plot(segs[:, 0, 0], segs[:, 0, 1], color=color, lw=1)
    for a, d in zip(heading + LIDAR_ANGLES, dists):
        plt.plot([origin[0], origin[0] + d * math.cos(a)], [origin[1], origin[1] + d * math.sin(a)],
                 color="tab:green", lw=0.5)
    plt.plot(*origin, "ro")
    plt.xlim(origin[0] - 60, origin[0] + 60)
    plt.ylim(origin[1] - 60, origin[1] + 60)
    plt.gca().set_aspect("equal")
    plt.title("LiDAR rays (green) vs left (blue) / right (orange) track edges")
    plt.savefig(path, dpi=120)
    plt.show()
    plt.close()
    env.close()


def plot_training_curve(log_path="./ppo_lidar_logs/evaluations.npz", path="training_curve.png"):
    results = np.load(log_path)
    mean_rewards = results["results"].mean(axis=1)
    std_rewards = results["results"].std(axis=1)

    plt.figure(figsize=(12, 5))
    plt.plot(results["timesteps"], mean_rewards, label="Mean eval reward", color="steelblue")
    plt.fill_between(results["timesteps"], mean_rewards - std_rewards, mean_rewards + std_rewards, alpha=0.2)
    plt.xlabel("Timesteps")
    plt.ylabel("Reward")
    plt.title("PPO + LiDAR Training Progress — CarRacing-v3")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=120)
    plt.show()
    plt.close()


def draw_lidar_on_frame(frame, distances):
    """Overlay LiDAR rays (metres) on an RGB frame from CarRacing's rgb_array render."""
    import cv2

    overlay = frame.copy()
    # The camera rotates with the car, so the car always points up. CarRacing
    # draws at ZOOM * SCALE = 16.2 px per world unit on a 1000x800 window with
    # the car at (W/2, 3H/4), then resizes that to the output frame.
    h, w = frame.shape[:2]
    px_x = 16.2 * w / 1000 * UNITS_PER_METER
    px_y = 16.2 * h / 800 * UNITS_PER_METER
    cx, cy = w // 2, int(h * 0.75 - LIDAR_OFFSET * px_y)

    for angle_offset, dist in zip(LIDAR_ANGLES, distances):
        screen_angle = math.pi / 2 + angle_offset
        ex = int(cx + dist * px_x * math.cos(screen_angle))
        ey = int(cy - dist * px_y * math.sin(screen_angle))  # y is flipped in pixel coords

        # Color: red (close) -> green (far)
        ratio = min(dist / LIDAR_MAX_RANGE, 1.0)
        color = (int(255 * (1 - ratio)), int(255 * ratio), 0)
        cv2.line(overlay, (cx, cy), (ex, ey), color, 1, cv2.LINE_AA)
        cv2.circle(overlay, (ex, ey), 2, color, -1)

    return cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)


def record_video(model, filename="car_lidar.mp4", max_decisions=400):
    """Drive one episode with the policy and save it with the LiDAR rays drawn on top."""
    import cv2

    env = make_car_env(randomize=False, render_mode="rgb_array")
    obs, _ = env.reset(seed=1)
    frames = []
    for _ in range(max_decisions):
        frames.append(draw_lidar_on_frame(env.render(), env.lidar_ranges()))
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break
    env.close()

    h, w = frames[0].shape[:2]
    out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"mp4v"), CONTRACT["control_rate"], (w, h))
    for frame in frames:
        out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    out.release()
    print(f"Saved {len(frames)} frames to {filename}")

    if IN_NOTEBOOK:
        import base64
        import subprocess
        from IPython.display import HTML, display

        # Re-encode with H.264 for browser playback
        subprocess.run(["ffmpeg", "-y", "-i", filename, "-vcodec", "libx264", "-f", "mp4",
                        "_playable.mp4", "-loglevel", "quiet"], check=True)
        with open("_playable.mp4", "rb") as f:
            mp4_data = base64.b64encode(f.read()).decode()
        display(HTML(f"""
            <video width="480" controls autoplay loop>
                <source src="data:video/mp4;base64,{mp4_data}" type="video/mp4">
            </video>
        """))


# ============================================================
# Train, evaluate, export
# ============================================================
def main():
    total_timesteps = int(os.environ.get("TIMESTEPS", 1_500_000))
    n_envs = max(2, os.cpu_count() or 2)

    plot_lidar_check()

    vec_cls = SubprocVecEnv if os.name == "posix" else DummyVecEnv
    train_env = vec_cls([make_env(randomize=True) for _ in range(n_envs)])
    eval_env = DummyVecEnv([make_env(randomize=False)])

    init_model = os.environ.get("INIT_MODEL")
    model = PPO.load(init_model, env=train_env, device="cpu") if init_model else PPO(
        "MlpPolicy",
        train_env,
        policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]), log_std_init=-0.5),
        learning_rate=3e-4,
        n_steps=4096 // n_envs,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        device="cpu",           # a small MLP trains faster on CPU than on GPU
        tensorboard_log="./ppo_lidar_tb/" if importlib.util.find_spec("tensorboard") else None,
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./ppo_lidar_best/",
        log_path="./ppo_lidar_logs/",
        eval_freq=max(1, 50_000 // n_envs),
        n_eval_episodes=5,
        deterministic=True,
        verbose=1,
    )

    model.learn(total_timesteps=total_timesteps, callback=eval_callback, progress_bar=IN_NOTEBOOK)
    model.save("ppo_lidar_final")

    best_path = "./ppo_lidar_best/best_model"
    best_model = PPO.load(best_path if os.path.exists(best_path + ".zip") else "ppo_lidar_final", device="cpu")
    mean_reward, std_reward = evaluate_policy(best_model, eval_env, n_eval_episodes=10, deterministic=True)
    print(f"Best model — Mean reward: {mean_reward:.1f} +/- {std_reward:.1f}")

    plot_training_curve()

    export_policy(best_model, "f1tenth_policy.npz")
    weights = np.load("f1tenth_policy.npz")
    obs = eval_env.reset()
    for _ in range(50):
        action, _ = best_model.predict(obs, deterministic=True)
        assert np.allclose(run_exported_policy(weights, obs[0]), action[0], atol=1e-4), \
            "exported weights do not reproduce the trained policy"
        obs, _, _, _ = eval_env.step(action)
    print("Exported f1tenth_policy.npz (verified against the trained model)")
    print("Contract:", json.dumps(CONTRACT, indent=2))

    try:
        record_video(best_model)
    except Exception as e:  # video needs opencv (and ffmpeg in a notebook); training does not
        print(f"Skipped video: {e}")


if __name__ == "__main__":
    main()
