"""Portable JSON scene recipes: analytic rooms or scanned floors plus rigid props."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import distance_transform_edt

from .objects import OBJECT_VERSION, add_object, fingerprint
from .scene import CACHE_VERSION, _numbers, prepare_scene

COMPOSITION_VERSION = 1


def _room(config, pitch):
    size = np.asarray(config.get("size", [6, 5, 2.6]), float)
    if size.shape != (3,) or not np.isfinite(size).all() or (size < [2, 2, 1.5]).any():
        raise ValueError("room.size must be [width, depth, height] >= [2,2,1.5] m")
    width, depth, height = size
    root = ET.Element("mujoco")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="room_floor", type="2d", builtin="checker", width="512", height="512",
                  rgb1="0.64 0.60 0.52", rgb2="0.69 0.65 0.57")
    ET.SubElement(asset, "material", name="room_floor", texture="room_floor", texrepeat="6 5", reflectance="0", specular="0")
    world = ET.SubElement(root, "worldbody")

    def box(name, pos, half, color, group=0, **extra):
        return ET.SubElement(world, "geom", name=name, type="box", pos=_numbers(pos), size=_numbers(half),
                             rgba=_numbers(color), group=str(group), contype="1", conaffinity="2",
                             friction="0.8 0.005 0.0001", **extra)

    box("room_floor", [0, 0, -.1], [width/2, depth/2, .1], [1, 1, 1, 1], material="room_floor")
    for label, xy, half in (("east", [width/2, 0], [.06, depth/2]),
                            ("west", [-width/2, 0], [.06, depth/2]),
                            ("north", [0, depth/2], [width/2, .06]),
                            ("south", [0, -depth/2], [width/2, .06])):
        color = [.72, .81, .84, 1] if label == "north" else [.88, .85, .79, 1]
        box(f"{label}_lower", [*xy, .65], [*half, .65], color)
        box(f"{label}_upper", [*xy, (height+1.3)/2], [*half, (height-1.3)/2], color, group=2)
    box("room_ceiling", [0, 0, height+.05], [width/2, depth/2, .05], [.94, .94, .91, 1], group=2)
    origin = -size[:2]/2
    shape = np.ceil(size[:2]/pitch).astype(int)
    xy = origin + (np.indices(shape).transpose(1, 2, 0)+.5)*pitch
    floor = (np.abs(xy) <= size[:2]/2).all(2)
    obstacles = (np.abs(xy) >= size[:2]/2-.06).any(2)
    nav = dict(floor=floor, obstacles=obstacles, origin=origin, resolution=np.array(pitch))
    metadata = dict(world_from_source=np.eye(4).tolist(), bounds=[[-width/2, -depth/2, 0], [width/2, depth/2, height]],
                    selected_floor=0, detected_up_axis="z", collision_boxes=10, triangles=0, visual_meshes=0)
    return root, nav, metadata


def prepare_composition(source, cache_root, *, force=False):
    source = Path(source).resolve()
    config = json.loads(source.read_text(encoding="utf-8"))
    if config.get("format") != "rsim-scene-v1":
        raise ValueError("composition format must be rsim-scene-v1")
    if ("room" in config) == ("base_scene" in config):
        raise ValueError("specify exactly one of room or base_scene")
    pitch = float(config.get("resolution", .08))
    if not .025 <= pitch <= .2:
        raise ValueError("resolution must be .025..0.2 m")
    specs = [dict(item) for item in config.get("objects", [])]
    names = [item["name"] for item in specs]
    if len(names) != len(set(names)):
        raise ValueError("object names must be unique")
    dependencies = []
    for spec in specs:
        spec["source"] = str((source.parent / spec["source"]).resolve())
        dependencies.append(fingerprint(spec["source"]))
    base = None
    if "base_scene" in config:
        base = (source.parent / config["base_scene"]).resolve()
        # Recipes reference raw scans; nesting would need recursive invalidation.
        if base.suffix.lower() != ".glb":
            raise ValueError("base_scene must be a self-contained .glb")
        dependencies.append(fingerprint(base))
    signature = dict(version=COMPOSITION_VERSION, object_version=OBJECT_VERSION, scan_version=CACHE_VERSION,
                     source=str(source), recipe=config, dependencies=dependencies)
    key = hashlib.sha256(json.dumps(signature, sort_keys=True).encode()).hexdigest()[:14]
    target = Path(cache_root).resolve() / f"{source.stem}-{key}"
    marker = target / "metadata.json"
    if marker.exists() and not force:
        return target
    target.mkdir(parents=True, exist_ok=True)
    marker.unlink(missing_ok=True)
    if base is None:
        root, nav, metadata = _room(config["room"], pitch)
    else:
        directory = prepare_scene(base, cache_root, up_axis=config.get("up_axis", "auto"),
                                  floor_height=config.get("floor_height"), pitch=pitch, force=force)
        for file in directory.iterdir():
            if file.suffix.lower() in (".obj", ".png"):
                shutil.copy2(file, target / file.name)
        root = ET.parse(directory / "scene.xml").getroot()
        with np.load(directory / "navigation.npz") as data:
            nav = {name: data[name].copy() for name in data.files}
        metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    objects = [add_object(root, target, spec, Path(cache_root) / "objects", force=force) for spec in specs]
    xy = nav["origin"] + (np.indices(nav["floor"].shape).transpose(1, 2, 0)+.5)*pitch
    for item in objects:
        # Conservative initial footprints avoid spawning through tables/seats.
        # Actual contact and sensing follow dynamic bodies, not this static map.
        yaw = item["yaw"]
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        local = (xy - item["position"][:2]) @ rotation
        half = np.array(item["extents"][:2])/2
        footprint = (np.abs(local) <= half + pitch/2).all(2)
        # Check the whole footprint, including unsupported/out-of-grid corners.
        corners = np.array([[-1,-1], [-1,1], [1,-1], [1,1]]) * half
        indices = np.floor((corners @ rotation.T + item["position"][:2]-nav["origin"])/pitch).astype(int)
        if (indices < 0).any() or (indices >= nav["floor"].shape).any() or not footprint.any():
            raise ValueError(f"{item['name']} footprint is outside the floor grid")
        if not nav["floor"][footprint].all() or nav["obstacles"][footprint].any():
            raise ValueError(f"{item['name']} overlaps an obstacle or unsupported floor; adjust position/yaw")
        nav["obstacles"] |= footprint
    nav["clearance"] = distance_transform_edt(np.pad(nav["floor"] & ~nav["obstacles"], 1))[1:-1, 1:-1]*pitch
    best = np.unravel_index(nav["clearance"].argmax(), nav["clearance"].shape)
    spawn = np.asarray(config.get("spawn", [*(nav["origin"]+(np.array(best)+.5)*pitch), 0]), float)
    if spawn.shape != (3,) or not np.isfinite(spawn).all():
        raise ValueError("spawn must be [x, y, yaw_radians]")
    ij = np.floor((spawn[:2]-nav["origin"])/pitch).astype(int)
    if (ij < 0).any() or (ij >= nav["clearance"].shape).any() or nav["clearance"][tuple(ij)] < .53:
        raise ValueError("recipe spawn lacks floor support or 0.53 m clearance")
    metadata.update(composition=signature, objects=objects, spawn=spawn.tolist(),
                    spawn_clearance=float(nav["clearance"][tuple(ij)]),
                    navigation_note="Initial object footprints; not updated as rigid bodies move.")
    ET.indent(root)
    ET.ElementTree(root).write(target / "scene.xml", encoding="utf-8", xml_declaration=True)
    np.savez_compressed(target / "navigation.npz", **nav)
    marker.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Composition ready: {source.stem}; {len(objects)} dynamic objects; {target}", flush=True)
    return target
