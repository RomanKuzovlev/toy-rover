from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("toy_rover")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_core_nodes = LaunchConfiguration("start_core_nodes")
    start_rviz = LaunchConfiguration("start_rviz")

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
        launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
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
            parameters=[{"use_sim_time": use_sim_time}],
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
        gazebo,
        robot_state_publisher,
        spawn_rover,
        bridge,
        map_to_odom_tf,
        rviz,
        *core_nodes,
    ])
