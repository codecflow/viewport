"""GLB extraction for Three.js viewers."""

import io
from dataclasses import dataclass

import mujoco
import trimesh

from . import geom


@dataclass
class Body:
    """Body with GLB mesh data."""

    id: int
    name: str
    glb: bytes
    fixed: bool


def extract(model: mujoco.MjModel) -> list[Body]:
    """Extract GLB meshes from MuJoCo model, one per body.

    Groups all visual geoms per body, merges into single mesh, exports as GLB.

    Args:
        model: MuJoCo model

    Returns:
        List of Body with GLB data
    """
    bodies = []

    for body_id in range(model.nbody):
        # Get visual geoms for this body
        geom_ids = geom.body_geom_ids(model, body_id, visual_only=True)

        if not geom_ids:
            continue  # No visual geoms

        # Group by texture compatibility and merge each group
        groups = geom.group_by_texture(model, geom_ids)
        meshes = []

        for group in groups:
            merged = geom.merge_geoms(model, group)
            meshes.append(merged)

        # Combine all groups for this body
        if len(meshes) == 1:
            body_mesh = meshes[0]
        else:
            body_mesh = trimesh.util.concatenate(meshes)

        # Export to GLB
        glb_bytes = _export_glb(body_mesh)

        bodies.append(
            Body(
                id=body_id,
                name=geom.body_name(model, body_id),
                glb=glb_bytes,
                fixed=geom.is_fixed(model, body_id),
            )
        )

    return bodies


def _export_glb(mesh: trimesh.Trimesh) -> bytes:
    """Export trimesh to GLB bytes."""
    with io.BytesIO() as buf:
        mesh.export(buf, file_type="glb")
        return buf.getvalue()
