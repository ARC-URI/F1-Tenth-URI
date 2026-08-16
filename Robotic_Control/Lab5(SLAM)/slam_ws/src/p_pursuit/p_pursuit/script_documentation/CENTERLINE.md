# Map Centerline Extractor

Extracts a smooth, closed centerline from a ROS/slam_toolbox occupancy grid map (`.pgm` + `.yaml`) and writes it out as a CSV of `x, y` world coordinates, plus a PNG overlay for visual verification.

## How it works

1. **Load** the `.pgm` image and its companion `.yaml` metadata (resolution, origin).
2. **Free-space mask** — threshold the image, keeping only pixels marked free (slam_toolbox convention: 254 = free, 205 = unknown, 0 = occupied).
3. **Morphological cleanup** — close then open the mask to fill small gaps and remove noise, using an elliptical kernel.
4. **Skeletonize** — reduce the free-space region to a 1-pixel-wide skeleton via Zhang-Suen thinning, then strip 2x2 pixel blocks and prune dead-end spurs so the skeleton reduces to a single loop.
5. **Walk the loop** — traverse the skeleton pixel-by-pixel, preferring the step that continues in the same direction, to produce an ordered path around the track.
6. **Convert to world coordinates** using the map's resolution and origin.
7. **Smooth** — bridge the gap between the path's start and end, close the loop, and fit a periodic B-spline to produce an evenly sampled, smooth centerline.
8. **Output** — write the centerline to CSV and render a red overlay of the centerline on top of the original map image for a sanity check.

## Requirements

```
opencv-contrib-python   # cv2.ximgproc requires the contrib build
numpy
pyyaml
scipy
```

Install with:

```bash
pip install opencv-contrib-python numpy pyyaml scipy
```

## Usage

```bash
python centerline.py --map ~/my_map --out ~/centerline.csv --overlay ~/centerline_overlay.png
```

`--map` is a path *stem* — the script expects `<stem>.pgm` and `<stem>.yaml` to both exist.

### Arguments

| Argument | Default | Description |
|---|---|---|
| `--map` | `~/my_map` | Path stem (no extension) for the `.pgm`/`.yaml` map pair. |
| `--out` | `~/centerline.csv` | Output CSV path for the smoothed centerline (`x`, `y` columns, world units). |
| `--overlay` | `~/centerline_overlay.png` | Output PNG showing the centerline drawn on the original map. |
| `--close-kernel` | `5` | Ellipse kernel size for morphological close/open. Increase for noisier walls or gaps in the map. |
| `--prune-iters` | `5` | Max spur-pruning passes on the skeleton. Increase to remove more dead-end branches; too high can eat into a nearly-closed loop. |
| `--samples` | `800` | Number of evenly spaced points in the final smoothed centerline. |
| `--smoothing` | `15.0` | `scipy.interpolate.splprep` smoothing factor `s`. `0` interpolates every point exactly; higher values smooth more and cut corners harder. |

## Output

- **CSV** (`--out`): header `x,y`, one row per sampled point, in the map's world frame (meters, per the map's `resolution`/`origin`).
- **Overlay PNG** (`--overlay`): the original grayscale map with the extracted centerline drawn in red, for a quick visual check that the loop was traced correctly.

## Tuning tips

- **Skeleton breaks into branches / "coverage" warning or error**: increase `--close-kernel` to smooth over wall noise, or increase `--prune-iters` to strip more spurs.
- **Centerline looks clipped near the start point**: lower `--prune-iters` — spur pruning may be eating into the loop itself.
- **Centerline cuts corners too aggressively**: lower `--smoothing` (try `0` for an exact fit through all points).
- **Centerline is jagged/noisy**: raise `--smoothing`.

## Notes

- Assumes a single closed-loop track (e.g., a racetrack map). Maps with multiple disconnected free-space regions or complex branching layouts are not supported by the loop-walking logic.
- The script exits with a `RuntimeError` if the walked path covers less than 50% of skeleton pixels, indicating the skeleton has unresolved branches.