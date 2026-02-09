"""GLB extraction for Genesis entities."""

import io
from dataclasses import dataclass

import trimesh


@dataclass
class Body:
    """Body with GLB mesh data."""
    id: int
    name: str
    glb: bytes
    fixed: bool


def extract(robot) -> list[Body]:
    """Extract GLB meshes from Genesis robot, one per vgeom.
    
    Args:
        robot: Genesis entity with vgeoms
        
    Returns:
        List of Body with GLB data
    """
    bodies = []
    
    for i, vgeom in enumerate(robot.vgeoms):
        try:
            mesh = vgeom.get_trimesh()
            
            # Export to GLB
            glb_bytes = _export_glb(mesh)
            
            bodies.append(Body(
                id=i,
                name=f"vgeom_{i}",
                glb=glb_bytes,
                fixed=vgeom.link.is_fixed,
            ))
        except Exception as e:
            print(f"Warning: Failed to extract vgeom {i}: {e}")
    
    return bodies


def _export_glb(mesh: trimesh.Trimesh) -> bytes:
    """Export trimesh to GLB bytes."""
    with io.BytesIO() as buf:
        mesh.export(buf, file_type="glb")
        return buf.getvalue()
