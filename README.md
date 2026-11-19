## Compute offline

In optimized_dp run. You can copy over the file from examples/turtlebot_4d_avoid_test.py in this repo to optimized_dp as it is slightly different:

```sh
conda activate odp
python examples/turtlebot_4d_avoid_test.py
```

## Build the package

Prerequsits: enter into environment with ros installed.

```sh
colcon build --packages-select my_robot_controller --symlink-install
```

## Launch nodes

Run the following command on the turtlebot:

```sh
ros2 launch turtlebot3_bringup robot.launch.py
```

Ensure your laptop can reach the robot. You can test with ros topic list command.

Use the launch command on your laptop to automatically, launch the 3 nodes (obstacle_detector, safety_filter, drive_foward).

This will drive the robot forward and for all clustered objects detected from the lidar sensor, look up the safety value. Then it picks the minimum and runs the safety filter on that object.

```sh
ros2 launch my_robot_controller safety_filter.launch.py
```

If you want Lidar infomation for debugging, launch visualizer node:

```sh
ros2 launch my_robot_controller obstacle_detector.launch.py
```