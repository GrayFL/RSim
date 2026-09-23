"""Hardware configuration values shared by providers and ROS-free clients."""
import json
import re


def d435_profile(value):
    pattern = r"\s*(\d+)\s*[xX,]\s*(\d+)\s*[xX,]\s*(\d+)\s*"
    match = re.fullmatch(pattern, value) if isinstance(value, str) else None
    if match is None or any(int(n) <= 0 for n in match.groups()):
        raise ValueError("camera profile must be WIDTHxHEIGHTxFPS with positive integers")
    return "x".join(str(int(n)) for n in match.groups())


def d435_config(serial, depth_profile, color_profile):
    if not isinstance(serial, str) or (serial and not re.fullmatch(r"[0-9]+", serial)):
        raise ValueError("serial must contain digits only (without the ROS '_' prefix)")
    return {"serial": serial, "depth_profile": d435_profile(depth_profile),
            "color_profile": d435_profile(color_profile)}


def d435_version(config):
    return "d435-v2:" + json.dumps(config, sort_keys=True)
