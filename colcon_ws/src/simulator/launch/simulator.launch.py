from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    # ROS2 args
    scenario_path_arg = DeclareLaunchArgument("scenario_desc", default_value="", description="...")
    simulation_data_arg = DeclareLaunchArgument("simulation_data", default_value="", description="...")
    
    scenario_path = LaunchConfiguration("scenario_desc")
    data_path = LaunchConfiguration("simulation_data")
            

    # get stonefish launch file
    stonefish_path = PathJoinSubstitution([
        FindPackageShare("stonefish_ros2"),
        "launch",
        "stonefish_simulator.launch.py"
        ]
    )

    # get Ardupilot ROS2 bridge launch file
    ardupilotros2_path = PathJoinSubstitution([
        FindPackageShare("ardupilotstonefish"),
        "launch",
        "parser_launch.yaml"
        ]
    )

    # get v4l2 ROS2 package launch file
    v4l2_path = PathJoinSubstitution([
        FindPackageShare("v4l2ros2"),
        "launch",
        "video_launch.py"
        ]
    )

    # get battery ROS2 package launch file
    battery_path = PathJoinSubstitution([
        FindPackageShare("battery"),
        "launch",
        "battery_launch.py"
        ]
    )

    # raytracer bash command
    path_raytracer = PathJoinSubstitution([FindPackagePrefix("anglerfish"), "lib", "anglerfish", "anglerfish"])
    raytracer_cmd = [path_raytracer,
                     scenario_path,
                     data_path]

    return LaunchDescription([
        scenario_path_arg,
        simulation_data_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stonefish_path),
            launch_arguments={"simulation_data": data_path,
                              "scenario_desc": scenario_path, 
                              "window_res_x": "1600",
                              "window_res_y": "1200",
                              "simulation_rate": "1000"}.items()
        ),  
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(ardupilotros2_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(v4l2_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(battery_path),
        ),
        ExecuteProcess(
            cmd = raytracer_cmd,
            output='screen'
        )
    ])
