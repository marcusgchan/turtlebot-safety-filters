## Compute offline

In optimized_dp run:

```sh
conda activate odp
python examples/turtlebot_4d_avoid_test.py
```

## Build the package

```sh
colcon build --packages-select my_robot_controller --symlink-install
```

## Launch nodes

Run the following command on the turtlebot:

```sh
ros2 launch turtlebot3_bringup robot.launch.py
```

Start up the drive_foward node and safety_filter node. You have 2 options. You can run in 2 seperate terminals or with 1 terminal using a launch command

### Launching with 2 terminals

Open 2 terminals and run the safety filter node in the first terminal:

```sh
ros2 run my_robot_controller safety_filter
```

Run the drive foward node in the second terminal:

```sh
ros2 run my_robot_controller drive_forward --ros-args -r cmd_vel:=cmd_vel_raw
```

### Using launch command

```sh
ros2 launch my_robot_controller safety_filter.launch.py
```