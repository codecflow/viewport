"""VisualScene extraction from Genesis entities."""

from __future__ import annotations

import numpy as np

from .extract import _quat_multiply, _quat_rotate
from .glb import Body


def _is_particle(entity) -> bool:
    """Check if entity has particle-based material (MPM, SPH, PBD)."""
    mat = getattr(entity, "material", None)
    if mat is None:
        return False
    cls = type(mat).__qualname__
    return any(t in cls for t in ("MPM", "SPH", "PBD", "Liquid", "Sand", "Elastic"))


def _is_articulated(entity) -> bool:
    """Check if entity is articulated (has vgeoms — MJCF/URDF robot)."""
    return bool(getattr(entity, "vgeoms", None))


def _entity_name(entity, idx: int) -> str:
    return getattr(entity, "name", None) or f"entity_{idx}"


def _get_rgba(entity) -> list[float]:
    """Try to get RGBA color from entity surface or material."""
    surface = getattr(entity, "surface", None)
    if surface is not None:
        c = getattr(surface, "color", None)
        if c is not None:
            c = list(c)
            return c if len(c) == 4 else c + [1.0]
    return [0.7, 0.7, 0.7, 1.0]


def _morph_geom(entity) -> dict | None:
    """Build inline GeomDesc from entity morph if it's a simple primitive."""
    morph = getattr(entity, "morph", None)
    if morph is None:
        return None
    cls = type(morph).__name__
    color = _get_rgba(entity)
    mat = {"color": [round(float(x), 4) for x in color]}

    if cls == "Box":
        size = list(getattr(morph, "size", (0.1, 0.1, 0.1)))
        return {"type": "box", "size": [round(float(x), 6) for x in size], "material": mat}
    if cls == "Sphere":
        r = float(getattr(morph, "radius", 0.1))
        return {"type": "sphere", "size": [round(r, 6)], "material": mat}
    if cls == "Cylinder":
        r = float(getattr(morph, "radius", 0.05))
        h = float(getattr(morph, "height", 0.2))
        return {"type": "cylinder", "size": [round(r, 6), round(h, 6)], "material": mat}
    if cls == "Capsule":
        r = float(getattr(morph, "radius", 0.05))
        h = float(getattr(morph, "height", 0.2))
        return {"type": "capsule", "size": [round(r, 6), round(h, 6)], "material": mat}
    if cls == "Plane":
        return {"type": "plane", "size": [10.0, 10.0], "material": mat}
    return None


def _get_transform(entity) -> tuple[list[float], list[float] | None]:
    """Get world pos and quat (wxyz) from entity."""
    pos = entity.get_pos()
    quat = entity.get_quat()
    if hasattr(pos, "cpu"):
        pos = pos.cpu().numpy()
    if hasattr(quat, "cpu"):
        quat = quat.cpu().numpy()
    pos = [round(float(x), 6) for x in pos]
    quat = [round(float(x), 6) for x in quat]
    # Skip identity quat
    identity = abs(quat[0] - 1.0) < 1e-5 and all(abs(quat[i]) < 1e-5 for i in range(1, 4))
    return pos, None if identity else quat


def _articulated_bodies(entity, glb_list: list[Body], base_url: str) -> list[dict]:
    """Convert articulated robot vgeoms to rigid bodies with GLB mesh refs."""
    result = []
    for i, vgeom in enumerate(entity.vgeoms):
        link_pos = vgeom.link.get_pos()
        link_quat = vgeom.link.get_quat()
        if hasattr(link_pos, "cpu"):
            link_pos = link_pos.cpu().numpy()
        if hasattr(link_quat, "cpu"):
            link_quat = link_quat.cpu().numpy()

        world_pos = link_pos + _quat_rotate(np.asarray(vgeom.init_pos, dtype=float), link_quat)
        world_quat = _quat_multiply(link_quat, np.asarray(vgeom.init_quat, dtype=float))

        glb_id = glb_list[i].id if i < len(glb_list) else i
        result.append({
            "kind": "rigid",
            "name": f"vgeom_{i}",
            "pos": [round(float(x), 6) for x in world_pos],
            "quat": [round(float(x), 6) for x in world_quat],
            "geoms": [{"type": "mesh", "size": [], "meshUrl": f"{base_url}/body/{glb_id}.glb"}],
        })
    return result


def extract(
    entities: list,
    glb_bodies_map: dict | None = None,
    base_url: str = "",
) -> dict:
    """Extract VisualScene from a list of Genesis entities.

    Args:
        entities: List of Genesis entities (robots, rigid bodies, MPM fluids, etc.)
        glb_bodies_map: Dict mapping articulated entity → list[Body] (from genesis.glb.extract).
                        Required for articulated robots; other entity types auto-detected.
        base_url: Base URL for GLB mesh references (e.g. "http://localhost:8080").
    """
    glb_bodies_map = glb_bodies_map or {}
    bodies: list[dict] = []
    particles: list[dict] = []

    for idx, entity in enumerate(entities):
        if _is_particle(entity):
            name = _entity_name(entity, idx)
            color = _get_rgba(entity)
            particles.append({
                "name": name,
                "material": {"color": [round(float(x), 4) for x in color]},
                "size": 0.01,
                "sizeAttenuation": True,
            })

        elif _is_articulated(entity):
            glb_list = glb_bodies_map.get(entity, [])
            bodies.extend(_articulated_bodies(entity, glb_list, base_url))

        else:
            # Simple rigid body — extract shape from morph
            geom = _morph_geom(entity)
            if geom is None:
                continue
            name = _entity_name(entity, idx)
            pos, quat = _get_transform(entity)
            body: dict = {"kind": "rigid", "name": name, "pos": pos, "geoms": [geom]}
            if quat:
                body["quat"] = quat
            bodies.append(body)

    result: dict = {"bodies": bodies, "lights": [], "cameras": [], "sensors": []}
    if particles:
        result["particles"] = particles
    return result
