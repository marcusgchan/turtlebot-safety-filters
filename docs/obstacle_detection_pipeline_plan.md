# Obstacle Detection Pipeline Plan

From raw `/scan` (`sensor_msgs/LaserScan`) to a list of circular obstacle estimates
(center, radius) for the HJ safety filter.

Scope: steps 1-4 of the sensing pipeline — filtering, frame conversion, clustering,
and per-cluster circle fitting. Tracking across frames (step 5) and the
`safety_filter.py` consumer-side relative-frame lookup are out of scope here.

## 1. Filter invalid readings

Drop entries in `ranges[]` that cannot represent a real obstacle before any
geometric processing happens.

- Reject `inf` / `NaN` (no return within `range_max`).
- Reject `r < range_min` (unreliable too-close returns).
- Reject `r > range_max` (shouldn't occur, but guard against driver quirks).
- Optional: reject isolated single-index spikes (one outlier reading between two
  `inf` neighbors) — likely sensor noise, not a real obstacle one beam wide.

Output: a reduced array of `(index, range)` pairs that survived filtering, carried
forward with their original angle index so step 2 can still compute angle.

## 2. Convert to Cartesian, transform into the working frame

Each surviving `(index, range)` becomes a point in the lidar's own frame, then gets
moved into whatever frame the safety filter's state is defined in.

```
angle_i   = angle_min + i * angle_increment
x_sensor  = range_i * cos(angle_i)
y_sensor  = range_i * sin(angle_i)
```

TF: look up the transform from the scan's `frame_id` (e.g. `base_scan`) to the
target frame (`odom`, since `safety_filter.py`'s state `(x, y, v, theta)` comes
from `/odom`) at the scan's `header.stamp`, and apply it to every point. Use
`tf2_ros`'s `TransformListener`/`Buffer`; do not assume `base_scan == base_link` —
there is a small fixed offset on turtlebot3 that matters at close range.

## 3. Cluster points into candidate obstacles

Group the transformed points so that each group corresponds to one physical
object, not one scan beam.

- **Pass 1 (cheap):** range-discontinuity clustering in the original angle order —
  start a new cluster whenever `|range_i - range_i-1|` exceeds a threshold
  (e.g. 0.15 m) or a filtered-out index breaks the run. O(n), works because
  `/scan` is already angle-ordered. Handle wrap-around (index 0 and index N-1 are
  angularly adjacent on a 360° scan).
- **Pass 2 (if pass 1 proves too fragile):** Euclidean/DBSCAN clustering on the
  Cartesian points directly, independent of angular adjacency.
- Discard clusters that are clearly not a single bounded object: too few points,
  or span too large an arc/chord to plausibly be `OBSTACLE_RADIUS`-sized
  (walls, long flat segments).

## 4. Fit a shape to each cluster

The HJ solve assumes a circular obstacle, so each surviving cluster is reduced to
`(center, radius)`.

- Simple estimate: centroid of the cluster's points as center; half the max
  pairwise point spread (or chord-based estimate) as radius.
- Better fit: least-squares circle fit (Kasa/algebraic method) over the cluster's
  points — more stable against partial occlusion of one side of the obstacle.
- Sanity check the fitted radius against `OBSTACLE_RADIUS` from the solved value
  function (or the nearest bucket, if multiple radii are precomputed) — reject
  fits too far off to be the modeled obstacle class.

Output of step 4: a list of `(center_x, center_y, radius)` in the target frame,
ready to hand to `safety_filter.py`'s per-obstacle relative-frame lookup.
