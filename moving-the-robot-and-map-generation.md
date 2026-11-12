
## How to move robot

After ensuring the robot has the same network and ROS_DOMAIN_ID run on the robot:

```sh
ros2 launch turtlebot3_bringup robot.launch.py
```


To control it using your laptop run:

```sh
ros2 run turtlebot3_teleop teleop_keyboard
```

## How to create map

Run the following command and drive the robot around to create the map:

```sh
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False
```

Save the map once you're satisfied:

```sh
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## How to run robot with the map and navigate

Run the following command. After running, you can set target points to move the robot:

```sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=$HOME/map.yaml
```