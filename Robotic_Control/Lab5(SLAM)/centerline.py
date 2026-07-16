#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.interpolate import splprep, splev


def load_map(pgm_path: Path, yaml_path: Path):
    with yaml_path.open() as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Failed to read {pgm_path}")
    return img, meta


def free_space_mask(img: np.ndarray, meta: dict, free_px: int = 250) -> np.ndarray:
    # slam_toolbox writes 254 for free, 205 for unknown, 0 for occupied.
    return ((img >= free_px).astype(np.uint8)) * 255


def clean_mask(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return m


def zhang_suen_skeleton(mask: np.ndarray) -> np.ndarray:
    return cv2.ximgproc.thinning(mask, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)


def strip_2x2_blocks(skel: np.ndarray) -> np.ndarray:
    s = (skel > 0).astype(np.uint8)
    down = np.roll(s, -1, axis=0)
    right = np.roll(s, -1, axis=1)
    downright = np.roll(down, -1, axis=1)
    block_topleft = (s & down & right & downright).astype(bool)
    s[block_topleft] = 0
    return (s * 255).astype(np.uint8)


def prune_spurs(skel: np.ndarray, iters: int) -> np.ndarray:
    # An "endpoint" is a skeleton pixel with exactly one skeleton neighbor.
    # Repeatedly deleting them shortens dead-end branches
    s = (skel > 0).astype(np.uint8)
    kern = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
    for _ in range(iters):
        neighbor_count = cv2.filter2D(s, -1, kern)
        endpoints = (s == 1) & (neighbor_count == 1)
        if not endpoints.any():
            break
        s[endpoints] = 0
    return (s * 255).astype(np.uint8)


def walk_loop(skel: np.ndarray) -> list:
    ys, xs = np.where(skel > 0)
    if len(ys) == 0:
        raise RuntimeError("Skeleton is empty after pruning")

    pixels = set(zip(ys.tolist(), xs.tolist()))
    offsets = [(-1, -1), (-1, 0), (-1, 1),
               (0, -1),           (0, 1),
               (1, -1),  (1, 0),  (1, 1)]

    start = (int(ys[0]), int(xs[0]))
    ordered = [start]
    visited = {start}
    prev_dir = None
    curr = start

    while True:
        r, c = curr
        candidates = []
        for dr, dc in offsets:
            n = (r + dr, c + dc)
            if n in pixels and n not in visited:
                candidates.append((n, (dr, dc)))

        if not candidates:
            break

        if len(candidates) == 1 or prev_dir is None:
            nxt, prev_dir = candidates[0]
        else:
            pdr, pdc = prev_dir
            # Dot product of (dr,dc) with prev_dir picks the most-continuing step.
            nxt, prev_dir = max(candidates,
                                key=lambda ic: ic[1][0] * pdr + ic[1][1] * pdc)

        ordered.append(nxt)
        visited.add(nxt)
        curr = nxt

    coverage = len(ordered) / len(pixels)
    if coverage < 0.5:
        raise RuntimeError(
            f"Loop walk covered {len(ordered)}/{len(pixels)} skeleton pixels "
            f"({coverage:.0%}) — the skeleton has real branches. Try a larger "
            "--close-kernel or more --prune-iters.")
    if coverage < 0.85:
        print(f"      warning: coverage {coverage:.0%} — skeleton has extra branches "
              "that were skipped, but main loop was followed.")

    return ordered


def pixels_to_world(pixels: list, meta: dict, img_height: int) -> np.ndarray:
    # In ROS map YAML, `origin` is the world coord of the pixel at (row=height-1, col=0),
    # so world_y grows as image row decreases.
    res = meta["resolution"]
    ox, oy, _ = meta["origin"]
    arr = np.asarray(pixels, dtype=float)
    rows, cols = arr[:, 0], arr[:, 1]
    x = ox + cols * res
    y = oy + (img_height - 1 - rows) * res
    return np.column_stack([x, y])


def bridge_start_end(xy: np.ndarray) -> np.ndarray:
    start = xy[0]
    end = xy[-1]
    gap = np.linalg.norm(end - start)
    step = float(np.mean(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    n_bridge = max(int(round(gap / step)), 1)
    if n_bridge > 1:
        bridge = np.linspace(end, start, n_bridge + 2)[1:-1]
        return np.vstack([xy, bridge])
    return xy


def smooth_closed(xy: np.ndarray, num_samples: int, smoothing: float) -> np.ndarray:
    xy = bridge_start_end(xy)
    xy_closed = np.vstack([xy, xy[:1]])  # per=True needs first == last
    tck, _ = splprep([xy_closed[:, 0], xy_closed[:, 1]], s=smoothing, per=True)
    u = np.linspace(0.0, 1.0, num_samples, endpoint=False)
    xs, ys = splev(u, tck)
    return np.column_stack([xs, ys])


def draw_overlay(img: np.ndarray, centerline_world: np.ndarray, meta: dict,
                 out_path: Path) -> None:
    res = meta["resolution"]
    ox, oy, _ = meta["origin"]
    h = img.shape[0]
    canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    pts_px = []
    for x, y in centerline_world:
        col = int(round((x - ox) / res))
        row = int(round(h - 1 - (y - oy) / res))
        pts_px.append((col, row))
    for i in range(len(pts_px)):
        cv2.line(canvas, pts_px[i], pts_px[(i + 1) % len(pts_px)], (0, 0, 255), 1)
    cv2.imwrite(str(out_path), canvas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=str(Path.home() / "my_map"),
                    help="Path stem (without extension) of the .pgm/.yaml pair.")
    ap.add_argument("--out", default=str(Path.home() / "centerline.csv"))
    ap.add_argument("--overlay", default=str(Path.home() / "centerline_overlay.png"))
    ap.add_argument("--close-kernel", type=int, default=5,
                    help="Ellipse kernel size for morphological close/open. Bigger = "
                         "more aggressive gap-filling on noisy walls.")
    ap.add_argument("--prune-iters", type=int, default=5,
                    help="Max endpoint-stripping passes. Bigger = more spur removal, "
                         "but too many will nibble the ends of a nearly-closed loop.")
    ap.add_argument("--samples", type=int, default=800,
                    help="Number of points on the smoothed output centerline.")
    ap.add_argument("--smoothing", type=float, default=15.0,
                    help="scipy splprep `s`. 0 = interpolate every point exactly; "
                         "higher = smoother, cuts corners more.")
    args = ap.parse_args()

    stem = Path(args.map)
    pgm = stem.with_suffix(".pgm")
    yml = stem.with_suffix(".yaml")

    print(f"[1/6] Loading {pgm.name} + {yml.name}")
    img, meta = load_map(pgm, yml)
    print(f"      shape={img.shape}, resolution={meta['resolution']} m/px")

    print("[2/6] Extracting free-space mask")
    mask = free_space_mask(img, meta)

    print(f"[3/6] Morphological cleanup (kernel={args.close_kernel})")
    mask = clean_mask(mask, args.close_kernel)

    print("[4/6] Skeletonizing (Zhang-Suen)")
    skel = zhang_suen_skeleton(mask)
    skel = strip_2x2_blocks(skel)
    skel = prune_spurs(skel, args.prune_iters)
    n_skel = int((skel > 0).sum())
    print(f"      skeleton pixels: {n_skel}")

    print("[5/6] Walking loop + converting to world coords")
    pixel_loop = walk_loop(skel)
    world_loop = pixels_to_world(pixel_loop, meta, img.shape[0])
    print(f"      {len(world_loop)} raw ordered points")

    print(f"[6/6] Smoothing (samples={args.samples}, s={args.smoothing})")
    centerline = smooth_closed(world_loop, args.samples, args.smoothing)

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for x, y in centerline:
            w.writerow([f"{x:.4f}", f"{y:.4f}"])
    print(f"      wrote {args.out}")

    draw_overlay(img, centerline, meta, Path(args.overlay))
    print(f"      wrote {args.overlay}")


if __name__ == "__main__":
    main()
