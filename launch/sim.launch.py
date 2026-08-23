import math
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _runtime_obstacle_sdf(obstacle_count, seed):
    if obstacle_count < 0 or obstacle_count > 200:
        raise ValueError("runtime_obstacle_count must be between 0 and 200")

    rng = random.Random(seed)
    obstacle_centers = []
    links = []
    attempts = 0

    while len(links) < obstacle_count and attempts < obstacle_count * 50:
        attempts += 1
        x = rng.uniform(-12.5, 12.5)
        y = rng.uniform(-12.5, 12.5)

        # Keep the rover's spawn area open and avoid heavily overlapping blocks.
        if math.hypot(x, y) < 2.0:
            continue
        if any(math.hypot(x - ox, y - oy) < 0.8 for ox, oy in obstacle_centers):
            continue

        obstacle_centers.append((x, y))
        obstacle_id = len(links)
        length = rng.uniform(0.5, 1.6)
        width = rng.uniform(0.18, 0.45)
        yaw = rng.uniform(-math.pi, math.pi)
        links.append(f"""
      <link name="obstacle_{obstacle_id}">
        <pose>{x:.3f} {y:.3f} 0.3 0 0 {yaw:.3f}</pose>
        <collision name="collision">
          <geometry><box><size>{length:.3f} {width:.3f} 0.6</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.3f} {width:.3f} 0.6</size></box></geometry>
          <material>
            <ambient>0.75 0.32 0.08 1</ambient>
            <diffuse>0.85 0.38 0.10 1</diffuse>
          </material>
        </visual>
      </link>""")

    return """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="runtime_obstacles">
    <static>true</static>
{links}
  </model>
</sdf>""".format(links="".join(links))


def _spawn_runtime_obstacles(context):
    obstacle_count = int(LaunchConfiguration("runtime_obstacle_count").perform(context))
    seed = int(LaunchConfiguration("runtime_obstacle_seed").perform(context))
    obstacle_sdf = _runtime_obstacle_sdf(obstacle_count, seed)

    return [Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_runtime_obstacles",
        output="screen",
        parameters=[{
            "world": "mini_maze",
            "string": obstacle_sdf,
            "name": "runtime_obstacles",
            "allow_renaming": False,
        }],
        condition=IfCondition(LaunchConfiguration("spawn_runtime_obstacles")),
    )]


def generate_launch_description():
    package_share = FindPackageShare("toy_rover")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_core_nodes = LaunchConfiguration("start_core_nodes")
    start_rviz = LaunchConfiguration("start_rviz")
    enable_stuck_recovery = LaunchConfiguration("enable_stuck_recovery")
    gz_args = LaunchConfiguration("gz_args")

    world = PathJoinSubstitution([package_share, "worlds", "mini_maze.world"])
    rover_xacro = PathJoinSubstitution([package_share, "urdf", "rover.urdf.xacro"])
    bridge_config = PathJoinSubstitution([package_share, "config", "ros_gz_bridge.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "config", "toy_rover.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", rover_xacro]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rover",
        output="screen",
        parameters=[
            {
                "topic": "robot_description",
                "name": "toy_rover",
                "allow_renaming": True,
                "x": 0.0,
                "y": 0.0,
                "z": 0.10,
            }
        ],
    )

    runtime_obstacles = OpaqueFunction(function=_spawn_runtime_obstacles)

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    core_nodes = [
        Node(
            package="toy_rover",
            executable="odom_tf_broadcaster_node",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="toy_rover",
            executable="mapping_node",
            name="mapping_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "enable_stuck_recovery": enable_stuck_recovery,
            }],
            condition=IfCondition(start_core_nodes),
        ),
        Node(
            package="toy_rover",
            executable="planner_node",
            name="planner_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(start_core_nodes),
        ),
        Node(
            package="toy_rover",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(start_core_nodes),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_core_nodes", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("enable_stuck_recovery", default_value="false"),
        DeclareLaunchArgument("gz_args", default_value=["-r -v 4 ", world]),
        DeclareLaunchArgument("spawn_runtime_obstacles", default_value="false"),
        DeclareLaunchArgument("runtime_obstacle_count", default_value="40"),
        DeclareLaunchArgument("runtime_obstacle_seed", default_value="7"),
        gazebo,
        robot_state_publisher,
        spawn_rover,
        runtime_obstacles,
        bridge,
        map_to_odom_tf,
        rviz,
        *core_nodes,
    ])
