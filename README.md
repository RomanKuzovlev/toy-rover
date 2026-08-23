# toy-rover

A small ROS 2 + Gazebo rover sandbox for learning the basics of robotics programming.

v0.1: The robot could be moved with manual commands in Gazebo.
v0.2: Simple A* path planning, autonomous driving toward the goal, basic obstacle avoidance, and on-the-fly goal selection in RViz.

The simulation generates a different runtime obstacle layout by default. The
printed seed can be reused to reproduce a layout, or supplied explicitly:

```bash
ros2 launch toy_rover sim.launch.py runtime_obstacle_count:=80 runtime_obstacle_seed:=12
```

Disable generated obstacles with `spawn_runtime_obstacles:=false`.
