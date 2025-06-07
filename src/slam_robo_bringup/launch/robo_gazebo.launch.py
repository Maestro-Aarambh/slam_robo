from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("slam_design_description"),
        "urdf",
        "slam_design.urdf.xacro"
    ])

    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare("slam_robo_bringup"),
        "config",
        "gazebo_bridge.yaml"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("slam_design_description"),
        "rviz",
        "slam_urdf_config.rviz"
    ])

    gz_world_path = PathJoinSubstitution([
        FindPackageShare("slam_robo_bringup"),
        "world",
        "test_world.sdf"
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_path])
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ])
            ]),
            launch_arguments={"gz_args": [gz_world_path, " -r"]}.items()
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description"]
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": gazebo_config_path}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        )
    ])
