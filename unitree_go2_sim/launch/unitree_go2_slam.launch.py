import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    unitree_go2_sim = FindPackageShare("unitree_go2_sim")
    go2_sdk = FindPackageShare("go2_sdk")
    
    slam_config = PathJoinSubstitution([go2_sdk, "config", "slam.yaml"])
    rviz_config = PathJoinSubstitution([go2_sdk, "config", "rviz.rviz"])
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    
    unitree_go2_description = FindPackageShare("unitree_go2_description")
    walled_world = PathJoinSubstitution([unitree_go2_description, "worlds", "walled_world.sdf"])

    # Include the simulation launch file
    # Disable rviz in sim launch, we launch it here
    # Disable publish_map_tf because SLAM handles it
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([unitree_go2_sim, "launch", "unitree_go2_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": "false", 
            "publish_map_tf": "false",
            "world": walled_world,
        }.items(),
    )
    
    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_config,
        }.items(),
    )
    
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        sim_launch,
        slam_toolbox,
        rviz_node,
    ])
