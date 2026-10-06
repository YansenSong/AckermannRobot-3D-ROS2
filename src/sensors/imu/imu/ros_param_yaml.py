#!/usr/bin/env python3
"""Formatting helpers for ROS 2 parameter YAML files."""


def format_double(value):
    """Format a float as a YAML scalar that ROS 2 reads back as a double.

    ``%.12g`` renders whole numbers as ``0`` or ``1`` with no decimal point.
    rcl_yaml_param_parser types those as INTEGER, so declaring the parameter as
    a double fails with InvalidParameterTypeException at startup. Keep a decimal
    point or an exponent so the scalar always parses as a double.
    """
    text = f"{float(value):.12g}"
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return text
