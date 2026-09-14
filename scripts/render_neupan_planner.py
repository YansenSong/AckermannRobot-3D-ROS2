#!/usr/bin/env python3
"""Render NeuPAN vehicle parameters from the project-level vehicle config."""

import argparse
import math
from pathlib import Path

import yaml


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def require_number(mapping, key, section):
    if key not in mapping:
        raise KeyError(f"Missing {section}.{key}")
    value = float(mapping[key])
    if not math.isfinite(value):
        raise ValueError(f"{section}.{key} must be finite")
    return value


def render(vehicle_path: Path, template_path: Path, output_path: Path):
    config = load_yaml(vehicle_path)
    planner = load_yaml(template_path)

    vehicle = config.get("vehicle", {})
    geometry = vehicle.get("geometry", {})
    planning = vehicle.get("planning", {})

    length = require_number(geometry, "length", "vehicle.geometry")
    width = require_number(geometry, "width", "vehicle.geometry")
    wheelbase = require_number(geometry, "wheelbase", "vehicle.geometry")
    ref_speed = require_number(planning, "reference_speed", "vehicle.planning")
    max_speed = require_number(planning, "max_forward_speed", "vehicle.planning")
    max_accel = require_number(planning, "max_acceleration", "vehicle.planning")
    max_steer_rate = require_number(planning, "max_steering_rate", "vehicle.planning")
    min_radius = require_number(
        planning, "minimum_turning_radius", "vehicle.planning"
    )

    for name, value in (
        ("length", length),
        ("width", width),
        ("wheelbase", wheelbase),
        ("max_forward_speed", max_speed),
        ("max_acceleration", max_accel),
        ("minimum_turning_radius", min_radius),
    ):
        if value <= 0.0:
            raise ValueError(f"{name} must be > 0")

    # NeuPAN's second max_speed component is the bicycle-model steering angle.
    steering_angle = math.atan(wheelbase / min_radius)

    planner["ref_speed"] = ref_speed
    robot = planner.setdefault("robot", {})
    robot["kinematics"] = "acker"
    robot["max_speed"] = [max_speed, steering_angle]
    robot["max_acce"] = [max_accel, max_steer_rate]
    robot["length"] = length
    robot["width"] = width
    robot["wheelbase"] = wheelbase

    ipath = planner.setdefault("ipath", {})
    ipath["min_radius"] = min_radius

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(planner, stream, sort_keys=False, allow_unicode=True)

    print(
        "Rendered NeuPAN vehicle config: "
        f"L={length:.3f}m W={width:.3f}m wheelbase={wheelbase:.3f}m "
        f"Rmin={min_radius:.3f}m steer={steering_angle:.3f}rad"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", required=True, type=Path)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    render(args.vehicle, args.template, args.output)


if __name__ == "__main__":
    main()
