"""Validation and protocol packing for isolated L20-10/11 index probes."""
import math


def index_positions_to_can(positions):
    """Pack [abduction, base, tip] into the index finger's 0x42 payload.

    The successful ROS2 GUI recording uses 255 for unused command fields.
    Feedback reports zero in those fields; feedback is not a command template.
    No other finger is represented in this payload.
    """
    values = list(positions)
    if len(values) != 3:
        raise ValueError("Index positions must be [abduction, base, tip]")
    if any(not math.isfinite(v) or not 0 <= v <= 255 or abs(v - round(v)) > 1e-6 for v in values):
        raise ValueError("Index positions must be integer protocol values in [0,255]")
    side, base, tip = (int(round(v)) for v in values)
    return [side, 255, base, 255, 255, tip]
