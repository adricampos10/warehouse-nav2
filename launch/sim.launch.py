"""Start my headless simulation: Nav2's TurtleBot3 bringup in one of my worlds.

Run inside the container:
    ros2 launch /ws/launch/sim.launch.py world_name:=warehouse viz:=true
    ros2 launch /ws/launch/sim.launch.py scenario:=/ws/scenarios/warehouse_shelf.yaml

world_name picks the Gazebo world and the map (see scripts/worlds.py) and
spawns the robot at that world's default start; scenario takes the world and
the start from a scenario file instead. viz:=true also starts foxglove_bridge
on port 8765, so I can watch it in Lichtblick.

Launch arguments are shared with the included launch files, so mine must not
reuse their names: Nav2's bringup already has "world" (the path to the SDF).
"""
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
from metrics import validate_scenario  # noqa: E402
from worlds import DEFAULT_WORLD, default_start, launch_args  # noqa: E402


def nav2_simulation(context):
    """Include Nav2's bringup with the world, map and spawn position resolved."""
    scenario_file = LaunchConfiguration("scenario").perform(context)
    if scenario_file:
        scenario = validate_scenario(yaml.safe_load(Path(scenario_file).read_text()))
        world, start = scenario["world"], scenario["start"]
    else:
        world = LaunchConfiguration("world_name").perform(context)
        start = default_start(world)

    bringup = Path(get_package_share_directory("nav2_bringup")) / "launch" / "tb3_simulation_launch.py"
    arguments = {"headless": "True", "use_rviz": "False", **launch_args(world, start, ROOT)}
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup)),
                                     launch_arguments=arguments.items())]


def generate_launch_description():
    foxglove = Path(get_package_share_directory("foxglove_bridge")) / "launch" / "foxglove_bridge_launch.xml"
    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value=DEFAULT_WORLD,
                              description="World name from scripts/worlds.py"),
        DeclareLaunchArgument("scenario", default_value="",
                              description="Scenario file; overrides world and start"),
        DeclareLaunchArgument("viz", default_value="false",
                              description="Also start foxglove_bridge for Lichtblick"),
        OpaqueFunction(function=nav2_simulation),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(foxglove)),
                                 launch_arguments={"port": "8765", "use_sim_time": "true"}.items(),
                                 condition=IfCondition(LaunchConfiguration("viz"))),
    ])
