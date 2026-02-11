from __future__ import annotations

from typing import TYPE_CHECKING

import mujoco
import numpy as np
import rerun as rr

from viewport.mujoco import geom

if TYPE_CHECKING:
    import trimesh


def quat_wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    """MuJoCo (w,x,y,z) → Rerun (x,y,z,w)"""
    return np.array([q[1], q[2], q[3], q[0]])


class RerunVisualizer:
    """Rerun visualizer for MuJoCo simulations.

    Uses the same extraction as the browser visualizer (geom.py),
    but streams to Rerun instead of the browser.
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.nbody = model.nbody
        self.frame = 0
        self.opened = False

    def open(self, app_id: str = "mujoco-sim", spawn: bool = True) -> None:
        """Initialize Rerun and log static geometry."""
        rr.init(app_id, spawn=spawn)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        # Extract all body geometry
        for body_id in range(self.nbody):
            body = self.model.body(body_id)
            name = body.name if body.name else f"body_{body_id}"

            # Get geoms for this body and merge
            ids = geom.body_geom_ids(self.model, body_id)
            if not ids:
                continue

            mesh = geom.merge_geoms(self.model, ids)

            # Log mesh geometry (static)
            self._log_mesh(f"world/bodies/{name}/mesh", mesh)

        self.opened = True

    def _log_mesh(self, path: str, mesh: trimesh.Trimesh) -> None:
        """Log trimesh as rr.Mesh3D."""
        vertices = np.array(mesh.vertices, dtype=np.float32)
        faces = np.array(mesh.faces, dtype=np.uint32)

        # Extract colors if available
        if mesh.visual is not None and hasattr(mesh.visual, "vertex_colors"):
            colors = np.array(mesh.visual.vertex_colors, dtype=np.uint8)  # type: ignore
        elif mesh.visual is not None and hasattr(mesh.visual, "face_colors"):
            # Expand face colors to vertex colors
            face_colors = np.array(mesh.visual.face_colors, dtype=np.uint8)  # type: ignore
            colors = face_colors[faces].reshape(-1, 4)
        else:
            colors = None

        # Extract normals if available
        normals = None
        if hasattr(mesh, "vertex_normals") and mesh.vertex_normals is not None:
            normals = np.array(mesh.vertex_normals, dtype=np.float32)

        # Log mesh
        if colors is not None:
            rr.log(
                path,
                rr.Mesh3D(
                    vertex_positions=vertices,
                    triangle_indices=faces,
                    vertex_colors=colors,
                    vertex_normals=normals,
                ),
                static=True,
            )
        else:
            rr.log(
                path,
                rr.Mesh3D(
                    vertex_positions=vertices,
                    triangle_indices=faces,
                    vertex_normals=normals,
                ),
                static=True,
            )

    def update(self, data: mujoco.MjData) -> None:
        """Update body transforms from simulation data."""
        if not self.opened:
            raise RuntimeError("Call open() before update()")

        # Set timeline
        rr.set_time("frame", sequence=self.frame)
        rr.set_time("sim_time", duration=data.time)

        # Update transforms
        for body_id in range(self.nbody):
            body = self.model.body(body_id)
            name = body.name if body.name else f"body_{body_id}"

            pos = data.xpos[body_id]
            quat = quat_wxyz_to_xyzw(data.xquat[body_id])

            rr.log(
                f"world/bodies/{name}",
                rr.Transform3D(
                    translation=pos,
                    quaternion=quat,  # type: ignore
                ),
            )

        self.frame += 1

    def close(self) -> None:
        """Close the Rerun session."""
        # Rerun doesn't need explicit cleanup
        pass
