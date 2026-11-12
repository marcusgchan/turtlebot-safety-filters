# Estimating `ACCEL_MIN`/`ACCEL_MAX` from `/odom`

How we measured the real acceleration bound used by `safety_filter.py` and
`optimized_dp/examples/turtlebot_4d_avoid_test.py`, since no accel spec is
published for the robot.

## 1. Record while commanding a step input

Start recording `/odom` and `/cmd_vel` together so the step command and the
resulting speed are on the same timeline:

```bash
ros2 bag record -o accel_test /odom /cmd_vel
```

In a second terminal, command a single step to max linear speed:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.22}}"
```

Let it run for a few seconds, then stop it:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"
```

(`--once` just publishes one message and exits — the robot holds the last
commanded velocity until you publish a new one, there's no auto-stop.)

Ctrl+C the `ros2 bag record` terminal to finish.

## 2. Extract `twist.twist.linear.x` from the bag

`ros2`/`rclpy` weren't available on this host (ROS2 runs in a container
here), so the bag (sqlite3 + CDR) was read with the pure-Python
[`rosbags`](https://pypi.org/project/rosbags/) library instead:

```bash
python3 -m venv /tmp/pdfvenv && /tmp/pdfvenv/bin/pip install rosbags
```

```python
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

typestore = get_typestore(Stores.ROS2_HUMBLE)
with AnyReader([Path("accel_test")], default_typestore=typestore) as reader:
    conns = [c for c in reader.connections if c.topic == "/odom"]
    t0 = None
    for conn, timestamp, rawdata in reader.messages(connections=conns):
        msg = reader.deserialize(rawdata, conn.msgtype)
        t0 = t0 or timestamp
        t = (timestamp - t0) / 1e9
        v = msg.twist.twist.linear.x
        print(f"{t:.3f}\t{v:.4f}")
```

This dumped `v(t)` for the whole recording as tab-separated `time  speed`.

## 3. Compute the slope (0 → 90% rise time)

Relevant samples around the step:

```
t=7.715s   v=0.019
t=7.742s   v=0.124
t=7.808s   v=0.128
t=7.848s   v=0.151
t=7.898s   v=0.251
```

Steady state after this oscillated noisily around 0.17–0.27 (averaging the
commanded 0.22) — a 10/90%-style rise time was used instead of time-to-100%
specifically because an exact "reaches target" crossing is ill-defined
against noise like that.

- 90% of target: `0.9 * 0.22 = 0.198 m/s`
- Linearly interpolate between the two samples straddling `0.198`:
  `t=7.848 (v=0.151)` and `t=7.898 (v=0.251)`
  ```
  fraction = (0.198 - 0.151) / (0.251 - 0.151) = 0.473
  t_90%    = 7.848 + 0.473 * 0.05 ≈ 7.871s
  ```
- Rise time: `Δt = 7.871 - 7.715 = 0.156s`
- Rise in speed: `Δv = 0.198 - 0.019 = 0.179 m/s`
- Average acceleration: `a = Δv / Δt ≈ 1.15–1.27 m/s²` (depending on exactly
  which raw points are used for interpolation)

## 4. Pick a bound with margin

This is an *average* slope over the transient, not necessarily the true
instantaneous acceleration at every point (a real step response is more
likely exponential-ish: steeper right after the step, tapering near the
target). To avoid ever telling the HJ solver the robot can accelerate faster
than it really can, we rounded down rather than using the raw measured
value:

```
ACCEL_MIN, ACCEL_MAX = -1.0, 1.0   # m/s^2
```

Applied in both:
- `src/my_robot_controller/my_robot_controller/safety_filter.py`
- `optimized_dp/examples/turtlebot_4d_avoid_test.py` (`uMin`/`uMax[0]`)

## Known gaps

- Only the acceleration direction (0 → max speed) was measured.
  Deceleration (`ACCEL_MIN`) was assumed symmetric, not measured separately.
- Only ~4 samples span the actual transition (~20-25 Hz odom rate), so the
  slope estimate is coarse.
- No curve fit was done; a cleaner estimate would fit an exponential
  `v(t) = v_target * (1 - e^(-t/τ))` to the samples and reason from τ
  instead of a two-point average slope.
