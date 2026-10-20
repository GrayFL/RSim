"""Convert movable GLB assets, retaining appearance independently of collision.

Object coordinates: source up-axis -> Z, XY bounds centered, bottom at Z=0.
Primitive colliders are specified in this frame BEFORE uniform scaling.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import trimesh

from .scene import _numbers

OBJECT_VERSION = 1


def fingerprint(path):
    path = Path(path).resolve()
    stat = path.stat()
    return dict(path=str(path), size=stat.st_size, mtime=stat.st_mtime_ns)


def _export_colored(part, target, name, limit):
    """Bake face colors to padded atlas tiles; OBJ has no portable vertex color."""
    original = part
    if len(part.faces) > limit:
        part = part.simplify_quadric_decimation(face_count=limit)
    # Sampling the source also preserves textured assets after decimation.
    visual = original.visual
    if visual.kind == "texture":
        colors = visual.to_color().vertex_colors[:, :3]
    else:
        colors = visual.vertex_colors[:, :3]
    distances, indices = cKDTree(original.vertices).query(part.vertices, k=min(3, len(original.vertices)))
    if indices.ndim == 1:
        sampled = colors[indices]
    else:
        weights = 1 / np.maximum(distances, 1e-9)
        sampled = (colors[indices] * weights[..., None]).sum(1) / weights.sum(1)[:, None]
    face_colors = np.clip(sampled[part.faces].mean(1), 0, 255).astype(np.uint8)
    side = int(np.ceil(np.sqrt(len(part.faces))))
    atlas = np.zeros((side*side, 3), np.uint8)
    atlas[:len(face_colors)] = face_colors
    # Each triangle samples one constant 4x4 tile, avoiding interpolation across
    # unrelated palette entries (ordinary vertex-color-to-UV conversion does).
    image = np.repeat(np.repeat(atlas.reshape(side, side, 3), 4, 0), 4, 1)
    Image.fromarray(image).save(target / f"{name}.png")
    index = np.arange(len(part.faces))
    uv = np.column_stack(((index % side + .5)/side, 1-(index//side + .5)/side))
    with (target / f"{name}.obj").open("w", encoding="ascii") as f:
        np.savetxt(f, part.vertices, fmt="v %.8f %.8f %.8f")
        np.savetxt(f, uv, fmt="vt %.8f %.8f")
        for i, face in enumerate(part.faces + 1, 1):
            f.write("f " + " ".join(f"{v}/{i}" for v in face) + "\n")
    return len(part.faces)


def prepare_object(source, cache_root, *, up_axis="y", scale=1.0,
                   target_height=None, max_faces=30000, force=False):
    source = Path(source).resolve()
    if up_axis not in ("y", "z") or not np.isfinite(scale) or scale <= 0:
        raise ValueError("object up_axis must be y/z and scale must be positive")
    if target_height is not None and (not np.isfinite(target_height) or target_height <= 0):
        raise ValueError("target_height must be positive")
    if isinstance(max_faces, bool) or not isinstance(max_faces, int) or max_faces < 100:
        raise ValueError("max_faces must be an integer >= 100 per mesh")
    signature = dict(version=OBJECT_VERSION, source=fingerprint(source), up_axis=up_axis,
                     scale=scale, target_height=target_height, max_faces=max_faces)
    key = hashlib.sha256(json.dumps(signature, sort_keys=True).encode()).hexdigest()[:14]
    target = Path(cache_root).resolve() / f"object-{source.stem}-{key}"
    metadata_path = target / "object.json"
    if metadata_path.exists() and not force:
        return target, json.loads(metadata_path.read_text(encoding="utf-8"))
    target.mkdir(parents=True, exist_ok=True)
    metadata_path.unlink(missing_ok=True)
    scene = trimesh.load_scene(source, process=False)
    rotation = np.eye(4)
    if up_axis == "y":
        rotation[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    parts = []
    for node in scene.graph.nodes_geometry:
        matrix, geometry = scene.graph[node]
        part = scene.geometry[geometry].copy()
        part.apply_transform(rotation @ matrix)
        part.update_faces(part.area_faces > 1e-12)
        if len(part.faces):
            parts.append(part)
    if not parts:
        raise ValueError(f"No triangles in {source}")
    bounds = np.array([np.min([p.bounds[0] for p in parts], 0), np.max([p.bounds[1] for p in parts], 0)])
    extents = bounds[1] - bounds[0]
    if extents[2] <= 0:
        raise ValueError("object has zero height")
    scale = scale * (target_height / extents[2] if target_height is not None else 1)
    offset = np.r_[-bounds.mean(0)[:2], -bounds[0, 2]]
    meshes, triangles = [], 0
    for i, part in enumerate(parts):
        part.apply_translation(offset)
        part.apply_scale(scale)
        name = f"part_{i:03d}"
        triangles += _export_colored(part, target, name, max_faces)
        meshes.append(name)
    info = dict(**signature, applied_scale=float(scale), source_extents_z_up=extents.tolist(),
                extents=(extents*scale).tolist(), source_triangles=sum(len(p.faces) for p in parts),
                visual_triangles=triangles, meshes=meshes)
    metadata_path.write_text(json.dumps(info, indent=2), encoding="utf-8")
    return target, info


def add_object(root, target, spec, cache_root, *, force=False):
    """Append one dynamic rigid body with explicit primitive collision parts."""
    name = spec["name"]
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
        raise ValueError("object name must be an ASCII identifier")
    mass = float(spec["mass"])
    position = np.asarray(spec["position"], float)
    yaw = float(spec.get("yaw", 0))
    friction = np.asarray(spec.get("friction", [.65, .005, .0001]), float)
    if not np.isfinite(mass) or mass <= 0 or position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("object mass must be positive and position finite [x,y,z]")
    if not np.isfinite(yaw) or friction.shape != (3,) or not np.isfinite(friction).all() or (friction < 0).any():
        raise ValueError("invalid yaw or friction")
    if not spec.get("colliders"):
        raise ValueError("movable objects require colliders")
    directory, info = prepare_object(spec["source"], cache_root, force=force,
                                    **{k: spec[k] for k in ("up_axis", "scale", "target_height", "max_faces") if k in spec})
    body = ET.SubElement(root.find("worldbody"), "body", name=name,
                         pos=_numbers(position), euler=f"0 0 {yaw}")
    ET.SubElement(body, "freejoint", name=f"{name}_free")
    asset = root.find("asset")
    import shutil
    for part in info["meshes"]:
        prefix = f"object_{name}_{part}"
        for extension in ("obj", "png"):
            shutil.copy2(directory / f"{part}.{extension}", target / f"{prefix}.{extension}")
        ET.SubElement(asset, "mesh", name=prefix, file=f"{prefix}.obj", inertia="shell")
        ET.SubElement(asset, "texture", name=prefix, file=f"{prefix}.png", type="2d")
        ET.SubElement(asset, "material", name=prefix, texture=prefix, specular="0", shininess="0")
        ET.SubElement(body, "geom", name=f"{prefix}_visual", type="mesh", mesh=prefix,
                      material=prefix, mass="0", group="0", contype="0", conaffinity="0")
    scale = info["applied_scale"]
    colliders = []
    for i, collider in enumerate(spec["colliders"]):
        kind = collider["type"]
        size = np.asarray(collider["size"], float)*scale
        if kind not in ("box", "capsule", "cylinder", "sphere"):
            raise ValueError(f"Unsupported collider: {kind}")
        expected = {"box": 3, "capsule": 1, "cylinder": 2, "sphere": 1}[kind]
        if size.shape != (expected,) or not np.isfinite(size).all() or (size <= 0).any():
            raise ValueError(f"Invalid collider size: {collider}")
        attributes = dict(name=f"{name}_collision_{i}", type=kind, size=_numbers(size),
                          group="3", contype="2", conaffinity="3", condim="4",
                          friction=_numbers(friction), rgba="0.1 0.8 0.2 0.4")
        if kind == "capsule":
            endpoints = np.asarray(collider["fromto"], float)*scale
            if endpoints.shape != (6,) or not np.isfinite(endpoints).all():
                raise ValueError("capsule fromto must have six finite numbers")
            attributes["fromto"] = _numbers(endpoints)
            volume = np.pi*size[0]**2*np.linalg.norm(endpoints[:3]-endpoints[3:]) + 4/3*np.pi*size[0]**3
        else:
            pos = np.asarray(collider.get("pos", [0, 0, 0]), float)*scale
            euler = np.asarray(collider.get("euler", [0, 0, 0]), float)
            if pos.shape != (3,) or euler.shape != (3,) or not np.isfinite([pos, euler]).all():
                raise ValueError("collider pos/euler must have three finite numbers")
            attributes.update(pos=_numbers(pos), euler=_numbers(euler))
            volume = (8*np.prod(size) if kind == "box" else
                      2*np.pi*size[0]**2*size[1] if kind == "cylinder" else 4/3*np.pi*size[0]**3)
        colliders.append((attributes, float(volume)))
    total_volume = sum(volume for _, volume in colliders)
    for attributes, volume in colliders:
        ET.SubElement(body, "geom", **attributes, mass=str(mass*volume/total_volume))
    return dict(name=name, **info, position=position.tolist(), yaw=yaw, mass=mass,
                friction=friction.tolist(), collision_parts=len(colliders))
