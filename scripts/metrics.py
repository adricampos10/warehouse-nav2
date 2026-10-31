"""Scenario checks and metrics, kept free of ROS so they can be unit-tested.

run_scenarios.py collects the raw data in the simulation (goals reached,
time, robot positions, laser ranges); the functions here turn it into
pass/fail results and a summary table.
"""
import math

DEFAULT_MIN_CLEARANCE = 0.15  # metres from the laser to the nearest obstacle
EXPECTATIONS = ("success", "failure")


def validate_scenario(data):
    """Check a scenario loaded from YAML and fill in the defaults."""
    if not isinstance(data, dict):
        raise ValueError("scenario must be a mapping")
    for key in ("start", "goals", "timeout"):
        if key not in data:
            raise ValueError(f"missing key: {key}")

    def point(value, what):
        if (not isinstance(value, (list, tuple)) or len(value) != 2
                or not all(isinstance(v, (int, float)) for v in value)):
            raise ValueError(f"{what} must be [x, y], got {value!r}")
        return (float(value[0]), float(value[1]))

    goals = data["goals"]
    if not isinstance(goals, list) or not goals:
        raise ValueError("goals must be a non-empty list")
    if not isinstance(data["timeout"], (int, float)) or data["timeout"] <= 0:
        raise ValueError("timeout must be a positive number of seconds")
    expect = data.get("expect", "success")
    if expect not in EXPECTATIONS:
        raise ValueError(f"expect must be one of {EXPECTATIONS}, got {expect!r}")

    return {
        "description": data.get("description", ""),
        "start": point(data["start"], "start"),
        "goals": [point(g, "goal") for g in goals],
        "timeout": float(data["timeout"]),
        "min_clearance": float(data.get("min_clearance", DEFAULT_MIN_CLEARANCE)),
        "expect": expect,
    }


def path_length(points):
    """Total distance travelled along a list of (x, y) positions."""
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def min_valid_range(ranges, range_min, range_max):
    """Shortest valid laser reading, or None if there is none.

    Readings outside [range_min, range_max], inf and NaN mean "no return"
    and are ignored.
    """
    valid = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
    return min(valid) if valid else None


def evaluate(scenario, run):
    """Decide whether a scenario passed.

    run is a dict with goals_reached, timed_out and min_range (None if no
    laser data was seen). Returns (passed, reason).
    """
    total = len(scenario["goals"])
    reached_all = run["goals_reached"] == total and not run["timed_out"]

    if scenario["expect"] == "failure":
        if reached_all:
            return False, "expected Nav2 to fail, but it reached the goal"
        return True, "failed as expected"

    if run["timed_out"]:
        return False, f"timed out after {run['goals_reached']}/{total} goals"
    if not reached_all:
        return False, f"reached {run['goals_reached']}/{total} goals"
    if run["min_range"] is not None and run["min_range"] < scenario["min_clearance"]:
        return False, (f"too close to an obstacle: {run['min_range']:.2f} m "
                       f"(limit {scenario['min_clearance']:.2f} m)")
    return True, "all goals reached"


def format_table(results):
    """Markdown table of scenario results (works in a terminal and on GitHub)."""
    def num(value, digits):
        return "-" if value is None else f"{value:.{digits}f}"

    lines = [
        "| Scenario | Result | Goals | Time (s) | Path (m) | Min clearance (m) | Notes |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in sorted(results, key=lambda r: r["name"]):
        lines.append(
            f"| {r['name']} | {'PASS' if r['passed'] else 'FAIL'} "
            f"| {r['goals_reached']}/{r['goals_total']} | {num(r['elapsed'], 1)} "
            f"| {num(r['path_length'], 2)} | {num(r['min_range'], 2)} | {r['reason']} |"
        )
    passed = sum(r["passed"] for r in results)
    lines.append("")
    lines.append(f"{passed}/{len(results)} scenarios passed")
    return "\n".join(lines)
