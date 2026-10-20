"""GLB -> textured visuals + conservative, single-level surface voxel collision.

The original triangle mesh is retained for sensing. Collision boxes approximate
only the selected storey. No building-wide convex hull or infinite ground plane
is used: floor support exists only where the source has a floor surface.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage
import trimesh

CACHE_VERSION = 3


def _numbers(values):
    return " ".join(f"{float(x):.8g}" for x in values)


def rectangles(mask):
    """Partition a boolean [x,y] array into nonoverlapping rectangles."""
    remaining = mask.copy()
    for x, y in zip(*np.nonzero(mask)):
        if not remaining[x, y]:
            continue
        y1 = y + 1
        while y1 < mask.shape[1] and remaining[x, y1]:
            y1 += 1
        x1 = x + 1
        while x1 < mask.shape[0] and remaining[x1, y:y1].all():
            x1 += 1
        remaining[x:x1, y:y1] = False
        yield int(x), int(x1), int(y), int(y1)


def _surface_points(mesh, face_mask, pitch):
    indices = np.flatnonzero(face_mask)
    # Bounded batches prevent large scans from allocating a full subdivided mesh.
    for start in range(0, len(indices), 1000):
        part = mesh.submesh([indices[start:start + 1000]], append=True, repair=False)
        vertices, _ = trimesh.remesh.subdivide_to_size(
            part.vertices, part.faces, max_edge=pitch / 2, max_iter=16)
        yield vertices


def _floor_height(mesh):
    heights = mesh.triangles_center[:, 2]
    horizontal = mesh.face_normals[:, 2] > 0.95
    if mesh.area_faces[horizontal].sum() < 2.0:
        horizontal = np.abs(mesh.face_normals[:, 2]) > 0.95
    bins = np.arange(mesh.bounds[0, 2] - 0.04, mesh.bounds[1, 2] + 0.12, 0.08)
    weights, _ = np.histogram(heights[horizontal], bins, weights=mesh.area_faces[horizontal])
    # A significant horizontal surface below a ceiling, with room for the mast.
    candidates = np.flatnonzero((weights > max(2.0, weights.max() * 0.15)) &
                               (bins[:-1] < mesh.bounds[1, 2] - 1.3))
    if not len(candidates):
        raise ValueError("No major floor detected; specify --floor-height in source up-axis metres")
    k = candidates[0]
    # Join neighboring bins on a noisy scan's same floor before the median.
    end = k + 1
    while end < len(weights) and weights[end] > max(0.5, weights.max()*0.03) and end-k < 3:
        end += 1
    selected = horizontal & (heights >= bins[k]) & (heights < bins[end])
    order = np.argsort(heights[selected])
    z, area = heights[selected][order], mesh.area_faces[selected][order]
    return float(z[np.searchsorted(np.cumsum(area), area.sum() / 2)])


def prepare_scene(source, cache_root="assets/mujoco/scenes", *, up_axis="auto",
                  floor_height=None, pitch=0.08, force=False):
    source = Path(source).expanduser().resolve()
    if source.is_dir():
        for required in ("scene.xml", "navigation.npz", "metadata.json"):
            if not (source / required).is_file():
                raise FileNotFoundError(source / required)
        return source
    if source.suffix.lower() == ".json":
        from .composition import prepare_composition
        return prepare_composition(source, cache_root, force=force)
    if not source.is_file():
        raise FileNotFoundError(source)
    if up_axis not in ("auto", "y", "z") or not 0.025 <= pitch <= 0.2:
        raise ValueError("up_axis must be auto/y/z; collision resolution must be 0.025..0.2 m")
    signature = dict(version=CACHE_VERSION, source=str(source), size=source.stat().st_size,
                     mtime=source.stat().st_mtime_ns, up_axis=up_axis,
                     floor_height=floor_height, pitch=pitch)
    key = hashlib.sha256(json.dumps(signature, sort_keys=True).encode()).hexdigest()[:14]
    target = Path(cache_root).resolve() / f"{source.stem}-{key}"
    metadata_path = target / "metadata.json"
    if metadata_path.exists() and not force:
        return target
    target.mkdir(parents=True, exist_ok=True)
    # A failed forced rebuild must never leave a cache that appears complete.
    metadata_path.unlink(missing_ok=True)
    print(f"Loading {source.name} ...", flush=True)
    scene = trimesh.load_scene(source, process=False)
    mesh = scene.to_mesh()
    if not len(mesh.faces):
        raise ValueError("Scene contains no triangles")
    if up_axis == "auto":
        scores = [mesh.area_faces[np.abs(mesh.face_normals[:, k]) > 0.95].sum() for k in (1, 2)]
        up_axis = ("y", "z")[int(np.argmax(scores))]
    transform = np.eye(4)
    if up_axis == "y":
        transform[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    mesh.apply_transform(transform)
    floor = _floor_height(mesh) if floor_height is None else float(floor_height)
    transform[2, 3] = -floor
    mesh.apply_translation([0, 0, -floor])
    print(f"Up axis: {up_axis}; selected source floor: {floor:.4f} m", flush=True)
    root = ET.Element("mujoco")
    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    textures = {}
    visual_parts = []
    for node in scene.graph.nodes_geometry:
        node_transform, geometry = scene.graph[node]
        part = scene.geometry[geometry].copy()
        part.apply_transform(transform @ node_transform)
        part.update_faces(part.area_faces > 1e-12)
        if not len(part.faces):
            continue
        # Group 2 contains upper walls/ceilings for an interactive cutaway.
        # Both groups remain enabled for RGBD and laser raycasting.
        upper = part.triangles_center[:, 2] > 1.3
        for mask, group in ((~upper, 0), (upper, 2)):
            if mask.any():
                visual_parts.append((part.submesh([np.flatnonzero(mask)], append=True, repair=False), group))
    for i, (part, group) in enumerate(visual_parts):
        name = f"scene_{i:03d}"
        uv = getattr(part.visual, "uv", None)
        if len(part.vertices) == 3:
            # MuJoCo requires four vertices even for non-colliding shell meshes.
            # Split the triangle in-plane, retaining its exact surface and UVs.
            vertices, faces = np.asarray(part.vertices), np.asarray(part.faces)
            center = vertices[faces[0]].mean(axis=0)
            if uv is not None:
                uv = np.vstack([uv, uv[faces[0]].mean(axis=0)])
            a, b, c = faces[0]
            part.vertices = np.vstack([vertices, center])
            part.faces = np.vstack([faces[1:], [[a, b, 3], [b, c, 3], [c, a, 3]]])
        material = getattr(part.visual, "material", None)
        material = material.to_simple() if hasattr(material, "to_simple") else material
        texture = getattr(material, "image", None)
        rgba = np.asarray(getattr(material, "diffuse", [210, 210, 210, 255]), float) / 255
        if texture is not None and uv is not None:
            texture = texture.convert("RGB")
            digest = hashlib.sha256(texture.tobytes()).hexdigest()
            if digest not in textures:
                texture_name = f"texture_{len(textures):03d}"
                texture.save(target / f"{texture_name}.png")
                ET.SubElement(assets, "texture", name=texture_name, type="2d",
                              file=f"{texture_name}.png")
                textures[digest] = texture_name
            ET.SubElement(assets, "material", name=name, texture=textures[digest],
                          specular="0", shininess="0", rgba="1 1 1 1")
        else:
            if getattr(part.visual, "kind", None) in ("vertex", "face"):
                rgba = np.mean(part.visual.vertex_colors, axis=0) / 255
            ET.SubElement(assets, "material", name=name, rgba=_numbers(rgba), specular="0")
        with (target / f"{name}.obj").open("w", encoding="ascii") as f:
            np.savetxt(f, part.vertices, fmt="v %.8f %.8f %.8f")
            if uv is not None:
                np.savetxt(f, uv, fmt="vt %.8f %.8f")
                for face in part.faces + 1:
                    f.write("f " + " ".join(f"{v}/{v}" for v in face) + "\n")
            else:
                np.savetxt(f, part.faces + 1, fmt="f %d %d %d")
        ET.SubElement(assets, "mesh", name=name, file=f"{name}.obj", inertia="shell")
        ET.SubElement(world, "geom", name=name, type="mesh", mesh=name, material=name,
                      contype="0", conaffinity="0", group=str(group), mass="0")

    origin = np.floor(mesh.bounds[0, :2] / pitch) * pitch - pitch
    shape = np.ceil((mesh.bounds[1, :2] - origin) / pitch).astype(int) + 2
    if np.prod(shape) > 4_000_000:
        raise ValueError("Scene grid is too large; increase collision resolution or crop the scene")
    floor_mask = np.zeros(shape, bool)
    center_z = mesh.triangles_center[:, 2]
    floor_faces = (np.abs(mesh.face_normals[:, 2]) > 0.9) & (np.abs(center_z) < 0.065)
    for points in _surface_points(mesh, floor_faces, pitch):
        ij = np.floor((points[:, :2] - origin) / pitch).astype(int)
        floor_mask[ij[:, 0], ij[:, 1]] = True
    # The selected level is planar; preserve unsupported areas/stair openings.
    floor_mask = ndimage.binary_closing(floor_mask, iterations=1)
    max_height = 1.16
    nz = int(np.ceil(max_height / pitch))
    voxels = np.zeros((*shape, nz), bool)
    triangles = mesh.triangles
    obstacle_faces = (triangles[:, :, 2].max(axis=1) > 0.08) & (triangles[:, :, 2].min(axis=1) < max_height)
    print("Building collision surface voxels ...", flush=True)
    for points in _surface_points(mesh, obstacle_faces, pitch):
        points = points[(points[:, 2] > 0.08) & (points[:, 2] < max_height)]
        ij = np.floor((points[:, :2] - origin) / pitch).astype(int)
        k = np.floor(points[:, 2] / pitch).astype(int)
        voxels[ij[:, 0], ij[:, 1], k] = True
    count = 0

    def box(lo, hi, kind):
        nonlocal count
        ET.SubElement(world, "geom", name=f"{kind}_{count}", type="box",
                      pos=_numbers((np.asarray(lo) + hi) / 2),
                      size=_numbers((np.asarray(hi) - lo) / 2), group="3",
                      contype="1", conaffinity="2", rgba="0.4 0.5 0.6 1",
                      friction="1 0.005 0.0001")
        count += 1

    for x0, x1, y0, y1 in rectangles(floor_mask):
        box([origin[0] + x0*pitch, origin[1] + y0*pitch, -0.10],
            [origin[0] + x1*pitch, origin[1] + y1*pitch, 0], "floor")
    # Merge equal rectangles vertically to keep MuJoCo's broadphase inexpensive.
    active = {}
    for k in range(nz + 1):
        current = set(rectangles(voxels[:, :, k])) if k < nz else set()
        for rect in list(active):
            if rect not in current:
                x0, x1, y0, y1 = rect
                k0 = active.pop(rect)
                box([origin[0]+x0*pitch, origin[1]+y0*pitch, k0*pitch],
                    [origin[0]+x1*pitch, origin[1]+y1*pitch, k*pitch], "obstacle")
        for rect in current:
            active.setdefault(rect, k)
    free = floor_mask & ~voxels.any(axis=2)
    clearance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * pitch
    best = np.unravel_index(np.argmax(clearance), clearance.shape)
    if clearance[best] < 0.53:
        raise ValueError("No safe 0.6 m robot spawn on this floor; choose another --floor-height")
    spawn_xy = origin + (np.array(best) + 0.5) * pitch
    # Face the longer free direction from the safe spawn.
    directions = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])
    lengths = []
    for direction in directions:
        steps = 0
        for step in range(1, int(max(shape))):
            index = np.array(best) + direction * step
            if np.any(index < 0) or np.any(index >= shape) or clearance[tuple(index)] < 0.48:
                break
            steps = step
        lengths.append(steps)
    yaw = int(np.argmax(lengths)) * np.pi / 2
    metadata = dict(**signature, detected_up_axis=up_axis, selected_floor=floor,
                    world_from_source=transform.tolist(), bounds=mesh.bounds.tolist(),
                    spawn=[*spawn_xy.tolist(), float(yaw)], spawn_clearance=float(clearance[best]),
                    visual_meshes=len(visual_parts), triangles=len(mesh.faces),
                    collision_boxes=count, grid_origin=origin.tolist())
    ET.indent(root)
    ET.ElementTree(root).write(target / "scene.xml", encoding="utf-8", xml_declaration=True)
    np.savez_compressed(target / "navigation.npz", floor=floor_mask, obstacles=voxels.any(axis=2),
                        clearance=clearance, origin=origin, resolution=pitch)
    metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Scene ready: {count} collision boxes; spawn {metadata['spawn']}", flush=True)
    return target
