"""VisualScene extraction from Genesis entities."""

from __future__ import annotations

import numpy as np

from .extract import _quat_multiply, _quat_rotate


def extract(robot, extra_bodies: list[dict] | None = None, base_url: str = "") -> dict:
    """Extract VisualScene dict from Genesis robot.

    Each vgeom becomes a body with meshUrl pointing to {base_url}/body/{i}.glb.
    Pass extra_bodies to append additional inline-primitive bodies (box, sphere, etc.)
    that don't have corresponding GLBs.
    """
    bodies: list[dict] = []

    for i, vgeom in enumerate(robot.vgeoms):
        link_pos = vgeom.link.get_pos()
        link_quat = vgeom.link.get_quat()

        if hasattr(link_pos, "cpu"):
            link_pos = link_pos.cpu().numpy()
        if hasattr(link_quat, "cpu"):
            link_quat = link_quat.cpu().numpy()

        world_pos = link_pos + _quat_rotate(np.asarray(vgeom.init_pos, dtype=float), link_quat)
        world_quat = _quat_multiply(link_quat, np.asarray(vgeom.init_quat, dtype=float))

        bodies.append({
            "name": f"vgeom_{i}",
            "pos": [round(float(x), 6) for x in world_pos],
            "quat": [round(float(x), 6) for x in world_quat],
            "geoms": [{"type": "mesh", "size": [], "meshUrl": f"{base_url}/body/{i}.glb"}],
        })

    if extra_bodies:
        bodies.extend(extra_bodies)

    return {"bodies": bodies, "lights": [], "cameras": [], "sensors": []}
