"""Convert MuJoCo geometries to trimesh format.

Ported from mjlab viewer/viser/conversions.py with viser dependency removed.
Handles all MuJoCo geom types with materials, textures, and UVs.
"""

import mujoco
import numpy as np
import trimesh
import trimesh.visual
import trimesh.visual.material
from mujoco import mj_id2name, mjtGeom, mjtObj
from PIL import Image


def geom_to_trimesh(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh:
    """Convert a MuJoCo geometry to trimesh.

    Args:
        model: MuJoCo model
        geom_id: Geometry index

    Returns:
        Trimesh object with materials/textures applied
    """
    geom_type = model.geom_type[geom_id]

    if geom_type == mjtGeom.mjGEOM_MESH:
        return mesh_geom_to_trimesh(model, geom_id)
    else:
        return primitive_mesh(model, geom_id)


def mesh_geom_to_trimesh(
    model: mujoco.MjModel, geom_id: int, verbose: bool = False
) -> trimesh.Trimesh:
    """Convert a MuJoCo mesh geometry to trimesh with textures.

    Handles texture coordinates, materials, and vertex duplication for proper UV mapping.

    Args:
        model: MuJoCo model
        geom_id: Geometry index
        verbose: Enable debug output

    Returns:
        Trimesh with texture/material applied if available
    """
    mesh_id = model.geom_dataid[geom_id]

    # Get mesh data ranges
    vert_start = int(model.mesh_vertadr[mesh_id])
    vert_count = int(model.mesh_vertnum[mesh_id])
    face_start = int(model.mesh_faceadr[mesh_id])
    face_count = int(model.mesh_facenum[mesh_id])

    # Extract vertices and faces
    vertices = model.mesh_vert[vert_start : vert_start + vert_count]
    faces = model.mesh_face[face_start : face_start + face_count]

    # Check for texture coordinates
    texcoord_adr = model.mesh_texcoordadr[mesh_id]
    texcoord_num = model.mesh_texcoordnum[mesh_id]

    if texcoord_num > 0:
        # Mesh has UVs - need to duplicate vertices per face
        texcoords = model.mesh_texcoord[texcoord_adr : texcoord_adr + texcoord_num]
        face_texcoord_idx = model.mesh_facetexcoord[
            face_start : face_start + face_count
        ]

        # Duplicate vertices for proper UV mapping
        new_vertices = vertices[faces.flatten()]
        new_uvs = texcoords[face_texcoord_idx.flatten()]
        new_faces = np.arange(face_count * 3).reshape(-1, 3)

        mesh = trimesh.Trimesh(vertices=new_vertices, faces=new_faces, process=False)

        # Apply material and texture
        matid = model.geom_matid[geom_id]
        if matid >= 0 and matid < model.nmat:
            rgba = model.mat_rgba[matid]
            texid = _get_texture_id(model, matid)

            if texid >= 0:
                # Has texture
                image = _extract_texture(model, texid, verbose)
                if image:
                    material = trimesh.visual.material.PBRMaterial(
                        baseColorFactor=rgba,
                        baseColorTexture=image,
                        metallicFactor=0.0,
                        roughnessFactor=1.0,
                    )
                    mesh.visual = trimesh.visual.TextureVisuals(
                        uv=new_uvs, material=material
                    )
                else:
                    # Fallback to vertex colors
                    rgba_255 = (rgba * 255).astype(np.uint8)
                    mesh.visual = trimesh.visual.ColorVisuals(
                        vertex_colors=np.tile(rgba_255, (len(new_vertices), 1))
                    )
            else:
                # Material but no texture
                rgba_255 = (rgba * 255).astype(np.uint8)
                mesh.visual = trimesh.visual.ColorVisuals(
                    vertex_colors=np.tile(rgba_255, (len(new_vertices), 1))
                )
        else:
            # Default color
            color = _default_color(model, geom_id)
            mesh.visual = trimesh.visual.ColorVisuals(
                vertex_colors=np.tile(color, (len(new_vertices), 1))
            )
    else:
        # No texture coordinates - simpler case
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)

        matid = model.geom_matid[geom_id]
        if matid >= 0 and matid < model.nmat:
            rgba = model.mat_rgba[matid]
            rgba_255 = (rgba * 255).astype(np.uint8)
            mesh.visual = trimesh.visual.ColorVisuals(
                vertex_colors=np.tile(rgba_255, (len(mesh.vertices), 1))
            )
        else:
            color = _default_color(model, geom_id)
            mesh.visual = trimesh.visual.ColorVisuals(
                vertex_colors=np.tile(color, (len(mesh.vertices), 1))
            )

    return mesh


def primitive_mesh(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh:
    """Create mesh for primitive geometry types.

    Supports: sphere, box, capsule, cylinder, plane, ellipsoid, hfield.

    Args:
        model: MuJoCo model
        geom_id: Geometry index

    Returns:
        Trimesh representation of the primitive
    """
    size = model.geom_size[geom_id]
    geom_type = model.geom_type[geom_id]
    rgba = model.geom_rgba[geom_id].copy()
    rgba_uint8 = (np.clip(rgba, 0, 1) * 255).astype(np.uint8)

    if geom_type == mjtGeom.mjGEOM_SPHERE:
        mesh = trimesh.creation.icosphere(radius=size[0], subdivisions=2)

    elif geom_type == mjtGeom.mjGEOM_BOX:
        mesh = trimesh.creation.box(extents=2.0 * size)

    elif geom_type == mjtGeom.mjGEOM_CAPSULE:
        mesh = trimesh.creation.capsule(radius=size[0], height=2.0 * size[1])

    elif geom_type == mjtGeom.mjGEOM_CYLINDER:
        mesh = trimesh.creation.cylinder(radius=size[0], height=2.0 * size[1])

    elif geom_type == mjtGeom.mjGEOM_PLANE:
        plane_x = 2.0 * size[0] if size[0] > 0 else 20.0
        plane_y = 2.0 * size[1] if size[1] > 0 else 20.0
        mesh = trimesh.creation.box((plane_x, plane_y, 0.001))

    elif geom_type == mjtGeom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=1.0)
        mesh.apply_scale(size)

    elif geom_type == mjtGeom.mjGEOM_HFIELD:
        mesh = _hfield_mesh(model, geom_id)
        if mesh:
            return mesh  # Already has vertex colors
        else:
            # Fallback
            mesh = trimesh.creation.box((1, 1, 0.01))

    else:
        raise ValueError(f"Unsupported geom type: {geom_type}")

    # Apply vertex colors
    vertex_colors = np.tile(rgba_uint8, (len(mesh.vertices), 1))
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=vertex_colors)
    return mesh


def merge_geoms(model: mujoco.MjModel, geom_ids: list[int]) -> trimesh.Trimesh:
    """Merge multiple geometries into single trimesh.

    Each geom is transformed by its local pose before merging.

    Args:
        model: MuJoCo model
        geom_ids: List of geometry indices to merge

    Returns:
        Single merged trimesh
    """
    meshes = []
    for geom_id in geom_ids:
        mesh = geom_to_trimesh(model, geom_id)

        # Apply geom's local transform
        pos = model.geom_pos[geom_id]
        quat = model.geom_quat[geom_id]  # wxyz format

        transform = np.eye(4)
        transform[:3, :3] = _quat_to_matrix(quat)
        transform[:3, 3] = pos

        mesh.apply_transform(transform)
        meshes.append(mesh)

    if len(meshes) == 1:
        return meshes[0]
    return trimesh.util.concatenate(meshes)


def group_by_texture(model: mujoco.MjModel, geom_ids: list[int]) -> list[list[int]]:
    """Group geometries by texture compatibility for safe merging.

    Prevents gray meshes when mixing TextureVisuals and ColorVisuals.

    Args:
        model: MuJoCo model
        geom_ids: List of geometry indices

    Returns:
        List of geometry ID groups, each safe to merge
    """
    groups: dict[int, list[int]] = {}
    for gid in geom_ids:
        tex_id = _get_geom_texture_id(model, gid)
        groups.setdefault(tex_id, []).append(gid)
    return list(groups.values())


def is_fixed(model: mujoco.MjModel, body_id: int) -> bool:
    """Check if body is fixed (welded to world, not mocap).

    Args:
        model: MuJoCo model
        body_id: Body index

    Returns:
        True if body is fixed and won't move
    """
    is_weld = model.body_weldid[body_id] == 0
    root_id = model.body_rootid[body_id]
    root_is_mocap = model.body_mocapid[root_id] >= 0
    return is_weld and not root_is_mocap


def body_name(model: mujoco.MjModel, body_id: int) -> str:
    """Get body name with fallback.

    Args:
        model: MuJoCo model
        body_id: Body index

    Returns:
        Body name or "body_{body_id}" if unnamed
    """
    name = mj_id2name(model, mjtObj.mjOBJ_BODY, body_id)
    return name if name else f"body_{body_id}"


def body_geom_ids(
    model: mujoco.MjModel, body_id: int, visual_only: bool = True
) -> list[int]:
    """Get all geometry IDs for a body.

    Args:
        model: MuJoCo model
        body_id: Body index
        visual_only: If True, only return visual geoms (groups 0-2)

    Returns:
        List of geometry indices
    """
    geom_ids = []
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] == body_id:
            if visual_only:
                # Only visual geoms (groups 0-2, skip collision group 3+)
                if model.geom_group[geom_id] < 3:
                    geom_ids.append(geom_id)
            else:
                geom_ids.append(geom_id)
    return geom_ids


def _get_texture_id(model: mujoco.MjModel, matid: int) -> int:
    """Get texture ID from material (RGB or RGBA role)."""
    texid = int(model.mat_texid[matid, int(mujoco.mjtTextureRole.mjTEXROLE_RGB)])
    if texid < 0:
        texid = int(model.mat_texid[matid, int(mujoco.mjtTextureRole.mjTEXROLE_RGBA)])
    return texid


def _extract_texture(
    model: mujoco.MjModel, texid: int, verbose: bool = False
) -> Image.Image | None:
    """Extract texture data as PIL Image."""
    tex_width = model.tex_width[texid]
    tex_height = model.tex_height[texid]
    tex_nchannel = model.tex_nchannel[texid]
    tex_adr = model.tex_adr[texid]
    tex_size = tex_width * tex_height * tex_nchannel

    tex_data = model.tex_data[tex_adr : tex_adr + tex_size]

    # Reshape and flip (MuJoCo uses OpenGL origin at bottom-left)
    if tex_nchannel == 1:
        tex_array = tex_data.reshape(tex_height, tex_width)
        tex_array = np.flipud(tex_array)
        return Image.fromarray(tex_array.astype(np.uint8), mode="L")
    elif tex_nchannel == 3:
        tex_array = tex_data.reshape(tex_height, tex_width, 3)
        tex_array = np.flipud(tex_array)
        return Image.fromarray(tex_array.astype(np.uint8))
    elif tex_nchannel == 4:
        tex_array = tex_data.reshape(tex_height, tex_width, 4)
        tex_array = np.flipud(tex_array)
        return Image.fromarray(tex_array.astype(np.uint8))
    else:
        if verbose:
            print(f"Unsupported texture channels: {tex_nchannel}")
        return None


def _default_color(model: mujoco.MjModel, geom_id: int) -> np.ndarray:
    """Get default color for geometry based on collision/visual type."""
    is_collision = (
        model.geom_contype[geom_id] != 0 or model.geom_conaffinity[geom_id] != 0
    )
    if is_collision:
        return np.array([204, 102, 102, 128], dtype=np.uint8)  # Red-ish
    else:
        return np.array([31, 128, 230, 255], dtype=np.uint8)  # Blue-ish


def _get_geom_texture_id(model: mujoco.MjModel, geom_id: int) -> int:
    """Get texture ID that will be used for this geom, or -1 if untextured."""
    if model.geom_type[geom_id] != mjtGeom.mjGEOM_MESH:
        return -1

    matid = model.geom_matid[geom_id]
    if matid < 0 or matid >= model.nmat:
        return -1

    texid = _get_texture_id(model, matid)
    if texid < 0:
        return -1

    mesh_id = model.geom_dataid[geom_id]
    if model.mesh_texcoordnum[mesh_id] <= 0:
        return -1

    return texid


def _quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert MuJoCo quaternion (wxyz) to 3x3 rotation matrix."""
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _hfield_mesh(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh | None:
    """Generate heightfield mesh with vertex colors."""
    hfield_id = model.geom_dataid[geom_id]
    if hfield_id < 0:
        return None

    nrow = model.hfield_nrow[hfield_id]
    ncol = model.hfield_ncol[hfield_id]
    sx, sy, sz, base = model.hfield_size[hfield_id]

    # Get heightfield data
    offset = sum(model.hfield_nrow[k] * model.hfield_ncol[k] for k in range(hfield_id))
    hfield = model.hfield_data[offset : offset + nrow * ncol].reshape(nrow, ncol)

    # Create mesh grid
    x = np.linspace(-sx, sx, ncol)
    y = np.linspace(-sy, sy, nrow)
    xx, yy = np.meshgrid(x, y)
    zz = hfield * sz

    vertices = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))

    # Create faces
    faces_list = []
    for r in range(nrow - 1):
        for c in range(ncol - 1):
            i0 = r * ncol + c
            i1 = i0 + 1
            i2 = i0 + ncol
            i3 = i2 + 1
            faces_list.append([i0, i1, i3])
            faces_list.append([i0, i3, i2])
    faces = np.array(faces_list, dtype=np.int64)

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)

    # Color by height (HSV gradient)
    zz_min, zz_max = zz.min(), zz.max()
    if zz_max > zz_min:
        normalized = (zz - zz_min) / (zz_max - zz_min)
    else:
        normalized = np.full_like(zz, 0.5)

    hue = 0.5 - normalized * 0.45
    saturation = 0.6 - normalized * 0.2
    value = 0.4 + normalized * 0.3

    # HSV to RGB
    chroma = value * saturation
    xc = chroma * (1 - np.abs((hue * 6) % 2 - 1))
    m = value - chroma

    hue_sector = (hue * 6).astype(int) % 6
    rc = np.zeros_like(hue)
    gc = np.zeros_like(hue)
    bc = np.zeros_like(hue)

    for sector in range(6):
        mask = hue_sector == sector
        if sector == 0:
            rc[mask], gc[mask] = chroma[mask], xc[mask]
        elif sector == 1:
            rc[mask], gc[mask] = xc[mask], chroma[mask]
        elif sector == 2:
            gc[mask], bc[mask] = chroma[mask], xc[mask]
        elif sector == 3:
            gc[mask], bc[mask] = xc[mask], chroma[mask]
        elif sector == 4:
            rc[mask], bc[mask] = xc[mask], chroma[mask]
        elif sector == 5:
            rc[mask], bc[mask] = chroma[mask], xc[mask]

    rc += m
    gc += m
    bc += m

    vertex_colors = np.column_stack(
        [
            (np.clip(rc, 0, 1) * 255).astype(np.uint8).ravel(),
            (np.clip(gc, 0, 1) * 255).astype(np.uint8).ravel(),
            (np.clip(bc, 0, 1) * 255).astype(np.uint8).ravel(),
            np.full(nrow * ncol, 255, dtype=np.uint8),
        ]
    )

    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=vertex_colors)
    return mesh
