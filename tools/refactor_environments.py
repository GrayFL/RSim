"""Create isolated source-linked validation environments without building wheels.

Run this tool with the populated development interpreter. Only explicitly
selected local projects and the dependency closure of their requested extras
are linked into fresh venvs. The development site-packages directory and its
.pth files are never added to the target interpreter's search path.
"""
from __future__ import annotations

import argparse
from collections import deque
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import sysconfig
import tomllib
import venv

from packaging.markers import default_environment
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.utils import canonicalize_name
from packaging.version import Version


PROFILES = {
    "core": {
        "requirements": ["graphmap[mapping,config]", "pyrobo", "pytest"],
        "absent": ["rsim", "mujoco", "mujocosim"],
    },
    "engine-only": {
        "requirements": ["mujocosim[engine]", "pytest"],
        "absent": ["rsim", "pyrobo"],
    },
    "provider": {
        "requirements": ["rsim[dds]", "mujocosim[rsim,engine]", "pytest", "psutil"],
        "absent": ["pyrobo"],
    },
    "consumer": {
        "requirements": ["rsim[dds]", "pyrobo[rsim]", "graphmap[mapping,config]",
                         "pytest", "psutil"],
        "absent": ["mujoco", "mujocosim"],
    },
    "extension-client": {
        "requirements": ["rsim[dds]", "mujocosim[rsim]", "pytest"],
        "absent": ["mujoco", "pyrobo"],
    },
}


def source_projects(arguments):
    projects = {}
    for argument in arguments:
        name, separator, location = argument.partition("=")
        if not separator:
            raise ValueError("--source must have the form name=/absolute/repository")
        name = canonicalize_name(name)
        location = Path(location).expanduser().resolve()
        project = tomllib.loads((location / "pyproject.toml").read_text())["project"]
        if canonicalize_name(project["name"]) != name:
            raise ValueError(f"{location}: project name does not match {name}")
        if name in projects:
            raise ValueError(f"duplicate source {name}")
        if not (location / name / "__init__.py").is_file():
            raise ValueError(f"{location}: expected explicit package {name}/__init__.py")
        projects[name] = {"path": location, "project": project}
    expected = {"rsim", "pyrobo", "mujocosim", "graphmap"}
    if projects.keys() != expected:
        raise ValueError(f"provide all four explicit source projects: {sorted(expected)}")
    return projects


def resolve(requirements, projects):
    """Resolve only against explicit sources and already installed distributions."""
    pending = deque((Requirement(value), "profile") for value in requirements)
    processed = set()
    local, external = {}, {}
    marker_environment = default_environment()
    while pending:
        requirement, parent = pending.popleft()
        name = canonicalize_name(requirement.name)
        if requirement.url:
            raise ValueError(f"{parent}: URL dependency is unsupported: {requirement}")
        if name in projects:
            source = projects[name]
            project = source["project"]
            version = project["version"]
            python_requirement = project.get("requires-python", "")
            available_extras = set(project.get("optional-dependencies", {}))
            local[name] = source
        else:
            try:
                distribution = metadata.distribution(name)
            except metadata.PackageNotFoundError as error:
                raise ValueError(f"{parent} requires {requirement}; absent from source interpreter") from error
            external[name] = distribution
            version = distribution.version
            python_requirement = distribution.metadata.get("Requires-Python", "")
            available_extras = set(distribution.metadata.get_all("Provides-Extra", []))
        if requirement.specifier and Version(version) not in requirement.specifier:
            raise ValueError(f"{parent} requires {requirement}; source version is {version}")
        if python_requirement and Version(marker_environment["python_full_version"]) not in SpecifierSet(python_requirement):
            raise ValueError(f"{name} requires Python {python_requirement}; current interpreter is {sys.version}")
        unknown = set(requirement.extras) - available_extras
        if unknown:
            raise ValueError(f"{name} has no requested extras: {sorted(unknown)}")
        for extra in {"", *requirement.extras}:
            if (name, extra) in processed:
                continue
            processed.add((name, extra))
            if name in local:
                declarations = (project.get("dependencies", []) if not extra else
                                project.get("optional-dependencies", {})[extra])
            else:
                declarations = external[name].requires or []
            for declaration in declarations:
                dependency = Requirement(declaration)
                if dependency.marker is None or dependency.marker.evaluate({**marker_environment, "extra": extra}):
                    pending.append((dependency, str(requirement)))
    return local, external


def link_path(source, target):
    source = source.resolve()
    if target.is_symlink() and target.resolve() == source:
        return
    if target.exists() or target.is_symlink():
        raise ValueError(f"dependency path collision at {target}, requested source {source}")
    target.symlink_to(source, target_is_directory=source.is_dir())


def create_environment(root, profile, projects):
    definition = PROFILES[profile]
    local, external = resolve(definition["requirements"], projects)
    directory = root / profile
    if directory.exists():
        raise ValueError(f"{directory} already exists; choose a new output root")
    venv.EnvBuilder(with_pip=False, symlinks=True, system_site_packages=False).create(directory)
    python = directory / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    paths = json.loads(subprocess.check_output([
        str(python), "-I", "-c", "import json,sysconfig; print(json.dumps(sysconfig.get_paths()))",
    ], text=True))
    site = Path(paths["purelib"])
    links = {}
    for name, distribution in sorted(external.items()):
        if not distribution.files:
            raise ValueError(f"{name}: file manifest unavailable; refusing an implicit site-packages link")
        roots = sorted({path.parts[0] for path in distribution.files
                        if path.parts and path.parts[0] not in {"..", "__pycache__"}})
        links[name] = []
        for entry in roots:
            source = Path(distribution.locate_file(entry))
            if entry.endswith(".pth"):
                raise ValueError(f"{name}: requires a .pth file; review its contents explicitly before validation")
            if not source.exists():
                raise ValueError(f"{name}: declared source path is missing: {source}")
            if entry in definition["absent"]:
                raise ValueError(f"{profile} dependency closure would expose forbidden module {entry}")
            link_path(source, site / entry)
            links[name].append({"target": str(site / entry), "source": str(source.resolve())})
    source_manifest = {}
    for name, source in sorted(local.items()):
        if name in definition["absent"]:
            raise ValueError(f"{profile} requires forbidden local project {name}")
        location, project = source["path"], source["project"]
        link_path(location / name, site / name)
        source_manifest[name] = {
            "path": str(location), "version": project["version"],
            "head": subprocess.check_output(["git", "-C", str(location), "rev-parse", "HEAD"], text=True).strip(),
            "status": subprocess.check_output(["git", "-C", str(location), "status", "--short"], text=True),
            "dependencies": project.get("dependencies", []),
            "extras": project.get("optional-dependencies", {}),
        }
    manifest = {
        "profile": profile, "python": str(python), "requirements": definition["requirements"],
        "absent": definition["absent"], "source_python": sys.executable,
        "source_site_packages": sysconfig.get_path("purelib"),
        "sources": source_manifest,
        "distributions": {name: {"version": dist.version, "paths": links[name]}
                          for name, dist in sorted(external.items())},
        "installation": "explicit source/distribution symlinks; no wheel, no system-site-packages, no .pth",
    }
    manifest_path = directory / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment["PYTHONNOUSERSITE"] = "1"
    probe = Path(__file__).with_name("refactor_environment_probe.py")
    result = subprocess.run([str(python), "-I", str(probe), str(manifest_path)],
                            cwd=root, env=environment, text=True, capture_output=True, timeout=60)
    (directory / "probe.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(f"{profile} validation failed; see {directory / 'probe.log'}")
    print(json.dumps({"profile": profile, "python": str(python), "probe": "passed",
                      "distributions": len(external), "sources": sorted(local)}), flush=True)
    return manifest_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", action="append", required=True, help="Explicit name=/path/to/repository (all four projects)")
    parser.add_argument("--root", required=True, type=Path, help="New generated environments directory")
    parser.add_argument("--profiles", nargs="+", choices=PROFILES, default=list(PROFILES))
    args = parser.parse_args()
    projects = source_projects(args.source)
    root = args.root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    for profile in args.profiles:
        create_environment(root, profile, projects)


if __name__ == "__main__":
    main()
