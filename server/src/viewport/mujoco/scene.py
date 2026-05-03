"""VisualScene extraction from MuJoCo model."""

import mujoco
import numpy as np
from mujoco import mj_id2name, mjtGeom, mjtObj

_PRIM: dict[int, str] = {
    int(mjtGeom.mjGEOM_BOX): "box",
    int(mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mjtGeom.mjGEOM_PLANE): "plane",
    int(mjtGeom.mjGEOM_ELLIPSOID): "sphere",
}


def _size(gtype: int, raw: np.ndarray) -> list[float]:
    if gtype == int(mjtGeom.mjGEOM_BOX):
        return [round(float(raw[i] * 2), 6) for i in range(3)]
    if gtype in (int(mjtGeom.mjGEOM_SPHERE), int(mjtGeom.mjGEOM_ELLIPSOID)):
        return [round(float(raw[0]), 6)]
    if gtype in (int(mjtGeom.mjGEOM_CYLINDER), int(mjtGeom.mjGEOM_CAPSULE)):
        return [round(float(raw[0]), 6), round(float(raw[1] * 2), 6)]
    if gtype == int(mjtGeom.mjGEOM_PLANE):
        hx = float(raw[0]) if raw[0] > 1e-6 else 10.0
        hy = float(raw[1]) if raw[1] > 1e-6 else 10.0
        return [round(hx, 6), round(hy, 6)]
    return [round(float(x), 6) for x in raw[:3]]


def _identity_quat(q: np.ndarray) -> bool:
    return abs(float(q[0]) - 1.0) < 1e-6 and float(np.max(np.abs(q[1:]))) < 1e-6


def extract(model: mujoco.MjModel, base_url: str = "") -> dict:
    """Extract VisualScene dict from MuJoCo model.

    Primitive geoms are described inline with type/size/color.
    Bodies with mesh geoms reference {base_url}/body/{body_id}.glb.
    """
    bodies = []
    lights = []

    for bid in range(model.nbody):
        name = mj_id2name(model, mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"

        geom_ids = [
            g for g in range(model.ngeom)
            if model.geom_bodyid[g] == bid and model.geom_group[g] < 3
        ]
        if not geom_ids:
            continue

        pos = [round(float(x), 6) for x in model.body_pos[bid]]
        quat_raw = model.body_quat[bid]  # wxyz

        has_mesh = any(model.geom_type[g] == int(mjtGeom.mjGEOM_MESH) for g in geom_ids)

        if has_mesh:
            geoms: list[dict] = [
                {"type": "mesh", "size": [], "meshUrl": f"{base_url}/body/{bid}.glb"}
            ]
        else:
            geoms = []
            for g in geom_ids:
                gtype = int(model.geom_type[g])
                if gtype not in _PRIM:
                    continue
                rgba = [round(float(x), 4) for x in model.geom_rgba[g]]
                geom: dict = {
                    "type": _PRIM[gtype],
                    "size": _size(gtype, model.geom_size[g]),
                    "material": {"color": rgba},
                }
                gpos = model.geom_pos[g]
                if float(np.max(np.abs(gpos))) > 1e-6:
                    geom["pos"] = [round(float(x), 6) for x in gpos]
                gquat = model.geom_quat[g]
                if not _identity_quat(gquat):
                    geom["quat"] = [round(float(x), 6) for x in gquat]
                geoms.append(geom)

        if not geoms:
            continue

        body: dict = {"name": name, "pos": pos, "geoms": geoms}
        if not _identity_quat(quat_raw):
            body["quat"] = [round(float(x), 6) for x in quat_raw]
        bodies.append(body)

    for lid in range(model.nlight):
        lname = mj_id2name(model, mjtObj.mjOBJ_LIGHT, lid) or f"light_{lid}"
        diffuse = [round(float(x), 4) for x in model.light_diffuse[lid][:3]]
        lights.append({
            "type": "directional",
            "name": lname,
            "pos": [round(float(x), 4) for x in model.light_pos[lid]],
            "dir": [round(float(x), 4) for x in model.light_dir[lid]],
            "color": diffuse,
            "intensity": 1.0,
        })

    return {"bodies": bodies, "lights": lights, "cameras": [], "sensors": []}
