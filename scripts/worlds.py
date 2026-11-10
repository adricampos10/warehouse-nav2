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
    # my home, from an iPhone scan; (0, 0) is where I started scanning
    "house": ("worlds/house.sdf.xacro", "maps/house.yaml", (0.0, 0.0)),
}

# Worlds that publish model poses even headless (their SDF always loads the
# scene broadcaster). Moving obstacles need them; Nav2's sandbox only loads it
# with the Gazebo GUI, so it can't have moving obstacles.
POSE_WORLDS = {"warehouse", "house"}

# Nav2 parameters I change for a world, on top of Nav2's defaults (applied with
# nav2_common's RewrittenYaml: every node that has the key gets the value).
PARAM_REWRITES = {
    # My home's walls come from a scan, so they are 25-50 cm thick. AMCL's
    # default laser model (likelihood field) only looks at how far each laser
    # point is from the nearest wall, so a point that lands inside a thick wall
    # looks perfect: the estimate slid into the walls, up to 3 m off in narrow
    # spots. The beam model follows each ray to the first wall instead.
    "house": {"laser_model_type": "beam"},
}


def default_start(world):
    return WORLDS[world][2]


def launch_args(world, start, root):
    """Arguments for tb3_simulation_launch.py: spawn the robot at start in world."""
    world_file, map_file, _ = WORLDS[world]
    args = {"x_pose": str(start[0]), "y_pose": str(start[1])}
    if world_file:
        args.update(world=f"{root}/{world_file}", map=f"{root}/{map_file}")
    return args
