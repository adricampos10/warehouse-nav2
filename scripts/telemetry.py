"""Telemetry: time series of a robot's run, written as CSV.

Every SAMPLE_PERIOD simulated seconds, run_scenario.py and run_mission.py
write one row per robot with what it was doing (which leg of its route),
how fast it went, how close it came to obstacles, how sure it was of where
it is (AMCL's covariance) and how wrong it actually was (AMCL against
Gazebo's true pose), and how much Nav2 struggled (recoveries, global paths).
A second, small file has one row per leg: when it started, when it ended
and how it ended.

CSV on purpose: any tool reads it, and it needs nothing beyond Python's
standard library (the simulation image has no pandas). Parquet would be
worth it with millions of rows; the analysis (pandas) runs elsewhere.

No ROS imports, so it is unit-tested with pytest; the ROS side only fills
in a Sample.
"""
import csv
import math
from dataclasses import asdict, dataclass, fields

SAMPLE_PERIOD = 0.5  # simulated seconds between rows


@dataclass
class Sample:
    """One row of telemetry. None means the value wasn't available yet."""
    t: float                    # simulated seconds since the run started
    robot_id: str
    run_id: str
    scenario: str
    leg: int                    # index of the goal or place the robot is driving to
    state: str                  # "driving", or how the leg ended
    x: float = None             # where the robot thinks it is (TF map->base_footprint)
    y: float = None
    gt_x: float = None          # where Gazebo says it really is
    gt_y: float = None
    loc_error: float = None     # distance between the two, metres
    std_x: float = None         # AMCL's uncertainty (standard deviations)
    std_y: float = None
    std_yaw: float = None
    speed: float = None         # m/s, from the wheel odometry
    clearance: float = None     # m from the robot's edge to the nearest laser point
    recoveries: int = None      # Nav2's recovery behaviours so far in this leg
    plans: int = 0              # global paths published so far (Nav2 replans every second)
    distance_remaining: float = None


COLUMNS = [f.name for f in fields(Sample)]
LEG_COLUMNS = ["robot_id", "run_id", "scenario", "leg", "goal_x", "goal_y",
               "start", "end", "duration", "result", "recoveries"]


def stds_from_covariance(cov):
    """Standard deviations of x, y and yaw from a ROS 6x6 pose covariance (36 numbers).

    The diagonal holds the variances of x, y, z, roll, pitch and yaw.
    """
    if len(cov) != 36:
        raise ValueError("a pose covariance has 36 values")
    return tuple(math.sqrt(max(cov[i], 0.0)) for i in (0, 7, 35))


def distance(a, b):
    """Distance between two (x, y) points, or None if one is missing."""
    if a is None or b is None:
        return None
    return math.dist(a, b)


def speed(vx, vy):
    return math.hypot(vx, vy)


def due(now, last, period=SAMPLE_PERIOD):
    """Whether it's time for the next row (now and last in seconds; last None = never)."""
    return last is None or now - last >= period


def rounded(value, digits=4):
    return round(value, digits) if isinstance(value, float) else value


class CsvLog:
    """Rows appended to a CSV file with a fixed header; values rounded to keep files small."""

    def __init__(self, path, columns):
        self.columns = columns
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()

    def write(self, row):
        self.writer.writerow({k: ("" if v is None else rounded(v)) for k, v in row.items()})

    def close(self):
        self.file.close()


class TelemetryLog:
    """The two files of a run: <name>.csv (time series) and <name>_legs.csv."""

    def __init__(self, directory, name):
        self.samples = CsvLog(directory / f"{name}.csv", COLUMNS)
        self.legs = CsvLog(directory / f"{name}_legs.csv", LEG_COLUMNS)

    def sample(self, sample):
        self.samples.write(asdict(sample))

    def leg(self, **row):
        missing = set(LEG_COLUMNS) - row.keys()
        if missing:
            raise ValueError(f"leg row is missing {sorted(missing)}")
        self.legs.write(row)

    def close(self):
        self.samples.close()
        self.legs.close()
