"""Extract meshes and transforms from Genesis simulations."""

from __future__ import annotations

import numpy as np
import trimesh


def meshes(robot) -> list[tuple[str, trimesh.Trimesh]]:
    """Extract visual geometry meshes from Genesis robot.

    Args:
        robot: Genesis entity with vgeoms (visual geometries)

    Returns:
        List of (name, trimesh) tuples, one per visual geometry
    """
    result = []

    for i, vgeom in enumerate(robot.vgeoms):
        try:
            mesh = vgeom.get_trimesh()

            # Keep in local space - transforms will be applied at runtime
            name = f"vgeom_{i}"
            result.append((name, mesh))

        except Exception as e:
            print(f"Warning: Failed to extract vgeom {i}: {e}")

    return result


def transforms(scene, robot) -> tuple[np.ndarray, np.ndarray]:
    """Extract current vgeom transforms from Genesis scene.

    Args:
        scene: Genesis scene with rigid solver
        robot: Genesis entity

    Returns:
        Tuple of (positions, quaternions):
        - positions: (N, 3) float32 array
        - quaternions: (N, 4) float32 array in wxyz format
    """
    positions = []
    quaternions = []

    # Compose link transforms with vgeom local offsets
    for vgeom in robot.vgeoms:
        # Get link transform (always updated during physics)
        link_pos = vgeom.link.get_pos()
        link_quat = vgeom.link.get_quat()

        # Convert to numpy if tensor
        if hasattr(link_pos, "cpu"):
            link_pos = link_pos.cpu().numpy()
        if hasattr(link_quat, "cpu"):
            link_quat = link_quat.cpu().numpy()

        # Get vgeom local offset
        local_pos = vgeom.init_pos
        local_quat = vgeom.init_quat

        # Compose: world_pos = link_pos + rotate(local_pos, link_quat)
        world_pos = link_pos + _quat_rotate(local_pos, link_quat)

        # Compose: world_quat = link_quat * local_quat
        world_quat = _quat_multiply(link_quat, local_quat)

        positions.append(world_pos)
        quaternions.append(world_quat)

    return np.array(positions, dtype=np.float32), np.array(
        quaternions, dtype=np.float32
    )


def _quat_rotate(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q (wxyz format)."""
    w, x, y, z = q
    vx, vy, vz = v

    # q * v * q^-1 (simplified for unit quaternions)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    return np.array(
        [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]
    )


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions (wxyz format)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def particles(entity) -> np.ndarray:
    """Extract particle positions from Genesis entity.

    Args:
        entity: Genesis entity with MPM/particle material

    Returns:
        (M, 3) float32 array of particle positions
    """
    particles_3d = entity.get_particles()

    if len(particles_3d) == 0:
        return np.array([], dtype=np.float32).reshape(0, 3)

    positions = particles_3d[0]

    # Convert to numpy if tensor
    if hasattr(positions, "cpu"):
        positions = positions.cpu().numpy()

    return positions.astype(np.float32)
