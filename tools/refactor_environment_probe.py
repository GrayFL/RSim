"""Verify package absence and record provenance in a source-linked test venv."""
from __future__ import annotations

import importlib
import importlib.metadata as metadata
import importlib.util
import json
import os
from pathlib import Path
import sys


def main():
    manifest = json.loads(Path(sys.argv[1]).read_text())
    forbidden = manifest["absent"]
    assert sys.prefix != sys.base_prefix, "probe must run in an isolated venv"
    assert manifest["source_site_packages"] not in sys.path, "full development packages leaked into sys.path"
    assert "include-system-site-packages = false" in (Path(sys.prefix) / "pyvenv.cfg").read_text().lower()
    missing = {name: importlib.util.find_spec(name) is None for name in forbidden}
    assert all(missing.values()), f"forbidden packages are importable: {missing}"
    provenance = {}
    for name in manifest["sources"]:
        module = importlib.import_module(name)
        provenance[name] = {"file": module.__file__, "resolved": str(Path(module.__file__).resolve())}
        assert Path(module.__file__).resolve().is_relative_to(manifest["sources"][name]["path"])
    if "graphmap" in provenance:
        from graphmap.pose import Pose
        import numpy as np
        np.testing.assert_allclose(Pose(yaw=np.pi / 2, degrees=False)([1, 0, 0]), [0, 1, 0], atol=1e-12)
    if manifest["profile"] == "core":
        from pyrobo.core.geometry import planar_state, pose2d
        np.testing.assert_allclose(planar_state(pose2d(1, 2, .3)), [1, 2, .3])
    assert all(name not in sys.modules for name in forbidden)
    report = {
        "profile": manifest["profile"], "sys.executable": sys.executable,
        "sys.prefix": sys.prefix, "sys.base_prefix": sys.base_prefix,
        "sys.path": sys.path, "absent": missing, "sources": provenance,
        "distributions": sorted((item.metadata["Name"], item.version) for item in metadata.distributions()),
        "runtime_environment": {name: value for name, value in os.environ.items()
                                if name.startswith("RSIM_") or name in {"CYCLONEDDS_URI", "MUJOCO_GL"}},
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
