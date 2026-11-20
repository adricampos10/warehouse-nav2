"""Unit tests for scripts/telemetry.py."""
import csv

import pytest

from telemetry import (COLUMNS, LEG_COLUMNS, Sample, TelemetryLog, distance, due, speed,
                       stds_from_covariance)


def test_stds_come_from_the_covariance_diagonal():
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = 0.04, 0.09, 0.01  # variances of x, y, yaw
    assert stds_from_covariance(cov) == pytest.approx((0.2, 0.3, 0.1))


def test_covariance_must_be_6x6():
    with pytest.raises(ValueError):
        stds_from_covariance([0.0] * 9)


def test_distance_and_speed():
    assert distance((0, 0), (3, 4)) == 5
    assert distance(None, (3, 4)) is None
    assert speed(0.3, 0.4) == pytest.approx(0.5)


def test_rows_are_due_every_period():
    assert due(0.0, None)
    assert not due(10.2, 10.0)
    assert due(10.5, 10.0)


def test_writes_the_time_series_and_the_legs(tmp_path):
    log = TelemetryLog(tmp_path, "warehouse_patrol")
    log.sample(Sample(t=0.5, robot_id="robot-01", run_id="r1", scenario="warehouse_patrol",
                      leg=0, state="driving", x=1.0, gt_x=1.1, loc_error=0.1234567, speed=0.26))
    log.leg(robot_id="robot-01", run_id="r1", scenario="warehouse_patrol", leg=0, goal_x=5.8,
            goal_y=-4.0, start=0.0, end=30.5, duration=30.5, result="succeeded", recoveries=0)
    log.close()

    with open(tmp_path / "warehouse_patrol.csv") as f:
        rows = list(csv.DictReader(f))
    assert list(rows[0]) == COLUMNS
    assert rows[0]["loc_error"] == "0.1235"  # rounded
    assert rows[0]["y"] == ""                # missing values are empty
    with open(tmp_path / "warehouse_patrol_legs.csv") as f:
        legs = list(csv.DictReader(f))
    assert list(legs[0]) == LEG_COLUMNS and legs[0]["result"] == "succeeded"


def test_a_leg_row_needs_every_column(tmp_path):
    log = TelemetryLog(tmp_path, "x")
    with pytest.raises(ValueError, match="missing"):
        log.leg(robot_id="robot-01", leg=0)
    log.close()


def test_columns_cover_the_roadmap():
    # mission state, speed, clearance, localisation quality, replans, recoveries
    for name in ("state", "leg", "speed", "clearance", "std_x", "loc_error", "plans", "recoveries"):
        assert name in COLUMNS
