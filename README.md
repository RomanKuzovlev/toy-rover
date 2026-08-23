# toy-rover

A small ROS 2 + Gazebo rover sandbox for learning the basics of robotics programming.

v0.1: The robot could be moved with manual commands in Gazebo.
v0.2: Simple A* path planning, autonomous driving toward the goal, basic obstacle avoidance, and on-the-fly goal selection in RViz.

The simulation can spawn deterministic runtime obstacles. Enable them and
change the layout or density with launch arguments:

```bash
ros2 launch toy_rover sim.launch.py spawn_runtime_obstacles:=true runtime_obstacle_count:=60 runtime_obstacle_seed:=12
```

They are disabled by default while testing the core navigation behavior.
