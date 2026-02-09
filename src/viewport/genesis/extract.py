"""Extract meshes and transforms from Genesis simulations."""

from __future__ import annotations

import numpy as np
import trimesh
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import genesis as gs


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
    """Extract current link transforms from Genesis scene.
    
    Args:
        scene: Genesis scene with rigid solver
        robot: Genesis entity
        
    Returns:
        Tuple of (positions, quaternions):
        - positions: (N, 3) float32 array
        - quaternions: (N, 4) float32 array in wxyz format
    """
    # Build link indices for all vgeoms
    link_indices = [vgeom.link.idx for vgeom in robot.vgeoms]
    
    # Batch calls to rigid solver
    all_positions = scene.rigid_solver.get_links_pos(links_idx=link_indices)
    all_quaternions = scene.rigid_solver.get_links_quat(links_idx=link_indices)
    
    # Handle single vs multi-environment
    if scene.n_envs == 0:
        positions = all_positions
        quaternions = all_quaternions
    else:
        # Use first environment
        positions = all_positions[0]
        quaternions = all_quaternions[0]
    
    # Convert to numpy if tensor
    if hasattr(positions, 'cpu'):
        positions = positions.cpu().numpy()
    if hasattr(quaternions, 'cpu'):
        quaternions = quaternions.cpu().numpy()
    
    return positions.astype(np.float32), quaternions.astype(np.float32)


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
    if hasattr(positions, 'cpu'):
        positions = positions.cpu().numpy()
    
    return positions.astype(np.float32)
