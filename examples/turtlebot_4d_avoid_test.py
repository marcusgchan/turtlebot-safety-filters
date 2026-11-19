"""
HJ-reachability safety-filter generation for a turtlebot3-style
differential-drive robot, 4D version (state (x, y, v, theta), controls
(accel, turn_rate)) using DubinsCar4D.

Solves the backward reachable tube (avoid set) for a static circular
obstacle sitting at the grid's own origin, so (x, y) here mean displacement
*relative to the obstacle*, not absolute position. This makes the solved
value function reusable for an obstacle detected anywhere at runtime: a
consumer transforms the robot's absolute pose into obstacle-relative
coordinates (robot_pos - obstacle_pos) before querying it, instead of this
being valid for only the one obstacle placement it was solved against.
Saves the value function + grid definition + gradients for use elsewhere
(e.g. a ROS2 safety-filter node on the real robot).
"""
import numpy as np

from odp.Grid import Grid
from odp.Shapes import CylinderShape
from odp.dynamics import DubinsCar4D
from odp.solver import HJSolver, computeSpatDerivArray
from odp.Plots import PlotOptions, visualize_plots

# ---------------------------------------------------------------------------
# 1. Grid, centered on the obstacle: x, y are symmetric around (0, 0) so the
#    obstacle sits at the origin and the robot can be solved for approaching
#    from any direction (an absolute-position grid like [-2,6]x[-3,3] with
#    the obstacle off-center only made sense for one fixed scenario). Range
#    chosen to cover plausible sensor/detection range in any direction.
#    Dims: x, y, v, theta.
# ---------------------------------------------------------------------------
grid = Grid(
    minBounds=np.array([-4.0, -4.0, -0.05, -np.pi]),
    maxBounds=np.array([4.0, 4.0, 0.22, np.pi]),
    dims=4,
    pts_each_dim=np.array([64, 64, 24, 36]),
    periodicDims=[3],
)

OBSTACLE_CENTER = np.array([0.0, 0.0])  # obstacle sits at the grid's own origin
OBSTACLE_RADIUS = 0.3

obstacle = CylinderShape(
    grid,
    ignore_dims=[2, 3],
    center=np.array([OBSTACLE_CENTER[0], OBSTACLE_CENTER[1], 0.0, 0.0]),
    radius=OBSTACLE_RADIUS,
)

# ---------------------------------------------------------------------------
# 2. Dynamics -- bounds should match the real robot (turtlebot3 burger:
#    ~0.22 m/s max linear, ~2.84 rad/s max angular). Turn rate is commanded
#    directly (matches cmd_vel.angular.z). Accel bound is measured from a
#    step response (0 -> 0.22 m/s) logged via /odom: ramp to 90% of target
#    took ~0.156s, i.e. ~1.27 m/s^2; using 1.0 to stay under the measured
#    value. Deceleration was not separately measured; assumed symmetric.
#    No disturbance modeled. Must match ACCEL_MIN/MAX in safety_filter.py.
# ---------------------------------------------------------------------------
car = DubinsCar4D(
    uMin=[-1.0, -2.84],
    uMax=[1.0, 2.84],
    dMin=[0.0, 0.0],
    dMax=[0.0, 0.0],
    uMode="max",
    dMode="min",
)

# ---------------------------------------------------------------------------
# 3. Solve the backward reachable tube (avoid set) once, offline.
# ---------------------------------------------------------------------------
lookback_length = 2.0
t_step = 0.05
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

compMethods = {"TargetSetMode": "minVWithV0"}
result = HJSolver(car, grid, obstacle, tau, compMethods, saveAllTimeSteps=False, accuracy="medium")
V = result  # same shape as grid.pts_each_dim

# Precompute the gradient everywhere once.
dVdx = computeSpatDerivArray(grid, V, deriv_dim=1, accuracy="medium")
dVdy = computeSpatDerivArray(grid, V, deriv_dim=2, accuracy="medium")
dVdv = computeSpatDerivArray(grid, V, deriv_dim=3, accuracy="medium")
dVdtheta = computeSpatDerivArray(grid, V, deriv_dim=4, accuracy="medium")

# Save V + grid definition + gradients so this can be reused elsewhere
# (e.g. a ROS2 safety-filter node) without resolving.
np.savez(
    "turtlebot_4d_avoid_value_function.npz",
    V=V,
    dVdx=dVdx,
    dVdy=dVdy,
    dVdv=dVdv,
    dVdtheta=dVdtheta,
    minBounds=grid.min,
    maxBounds=grid.max,
    pts_each_dim=grid.pts_each_dim,
    periodicDims=np.array(grid.pDim),
    obstacle_center=OBSTACLE_CENTER,
    obstacle_radius=OBSTACLE_RADIUS,
)
print("Saved value function, gradients, and grid definition to turtlebot_4d_avoid_value_function.npz")

# ---------------------------------------------------------------------------
# 4. Visualize the avoid set (V=0 isosurface) over (x, y, theta), sliced at
#    a fixed speed index (can't plot all 4 dims at once).
# ---------------------------------------------------------------------------
v_slice_idx = grid.pts_each_dim[2] // 2
po = PlotOptions(
    do_plot=True,
    plot_type="set",
    plotDims=[0, 1, 3],
    slicesCut=[v_slice_idx],
    save_fig=True,
    filename="turtlebot_4d_avoid_set.png",
)
visualize_plots(V, grid, po)
