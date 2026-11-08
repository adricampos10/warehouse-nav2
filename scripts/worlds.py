"""The worlds a scenario can run in, and the launch arguments for each one.

tb3_simulation_launch.py takes the Gazebo world, the Nav2 map and the spawn
position as arguments, so switching worlds needs no change to Nav2 itself.
launch/sim.launch.py uses this module; it has no ROS imports, so it is
unit-tested with pytest.
"""
DEFAULT_WORLD = "tb3_sandbox"

# world name -> (world file, map file, default start), files relative to the repo root.
# None means Nav2's own TurtleBot3 sandbox world and map.
WORLDS = {
    "tb3_sandbox": (None, None, (-2.0, -0.5)),
    "warehouse": ("worlds/warehouse.sdf.xacro", "maps/warehouse.yaml", (-5.5, 0.0)),
}

# Worlds that publish model poses even headless (their SDF always loads the
# scene broadcaster). Moving obstacles need them; Nav2's sandbox only loads it
# with the Gazebo GUI, so it can't have moving obstacles.
POSE_WORLDS = {"warehouse"}


def default_start(world):
    return WORLDS[world][2]


def launch_args(world, start, root):
    """Arguments for tb3_simulation_launch.py: spawn the robot at start in world."""
    world_file, map_file, _ = WORLDS[world]
    args = {"x_pose": str(start[0]), "y_pose": str(start[1])}
    if world_file:
        args.update(world=f"{root}/{world_file}", map=f"{root}/{map_file}")
    return args
