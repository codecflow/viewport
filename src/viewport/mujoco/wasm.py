"""Export MuJoCo models to static WASM-powered sites."""

import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import mujoco


def export(
    model: mujoco.MjModel,
    output: str | Path,
    title: str = "MuJoCo Simulation",
    timestep: Optional[float] = None,
    xml_path: Optional[str | Path] = None,
) -> None:
    """Export MuJoCo model to static WASM site.
    
    Creates a standalone HTML site that runs physics in the browser using
    MuJoCo WASM. No Python server required.
    
    Args:
        model: MuJoCo model to export
        output: Output directory path
        title: HTML page title
        timestep: Simulation timestep (default: model.opt.timestep)
        xml_path: Path to original XML file (for resolving mesh/texture assets)
    
    Example:
        >>> model = mujoco.MjModel.from_xml_path("scene.xml")
        >>> wasm.export(model, output="dist/", xml_path="scene.xml")
        >>> # Open dist/index.html in browser
    """
    output_dir = Path(output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Exporting MuJoCo WASM site to {output_dir}")
    
    # Save model XML
    model_path = output_dir / "scene.xml"
    mujoco.mj_saveLastXML(str(model_path), model)
    scene_xml = model_path.read_text()
    print(f"✓ Wrote scene XML to {model_path}")
    
    # Get base directory for resolving asset paths
    xml_base_dir = None
    if xml_path:
        xml_base_dir = Path(xml_path).parent.absolute()
        print(f"  Using base directory: {xml_base_dir}")
    
    # Parse XML and copy external assets
    mesh_files, texture_files = _copy_external_assets(
        scene_xml, xml_base_dir, output_dir
    )
    
    # Rewrite XML paths to use local directories
    updated_xml = _rewrite_xml_paths(scene_xml, mesh_files, texture_files)
    model_path.write_text(updated_xml)
    
    # Create manifest
    all_files = ["scene.xml"] + mesh_files + texture_files
    manifest = {
        "title": title,
        "timestep": float(timestep or model.opt.timestep),
        "scene": "scene.xml",
        "files": all_files,
    }
    
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(f"✓ Wrote manifest with {len(all_files)} files")
    
    # Copy WASM template
    template_dir = Path(__file__).parent.parent / "visualizer" / "wasm"
    if not template_dir.exists():
        raise FileNotFoundError(
            f"WASM template not found at {template_dir}. "
            "The template should be in src/viewport/visualizer/wasm/"
        )
    
    for file in template_dir.glob("*"):
        if file.is_file():
            shutil.copy(file, output_dir / file.name)
    
    print(f"\n✓ Export complete: {output_dir.absolute()}")
    print(f"  Open {output_dir.absolute()}/index.html in a browser")


def _parse_xml_assets(xml_content: str) -> tuple[Optional[str], Optional[str], list[str], list[str]]:
    """Parse XML to extract asset information."""
    try:
        root = ET.fromstring(xml_content)
    except ET.ParseError:
        return None, None, [], []
    
    # Get meshdir and texturedir
    meshdir = None
    texturedir = None
    compiler = root.find("compiler")
    if compiler is not None:
        meshdir = compiler.get("meshdir", "").strip()
        texturedir = compiler.get("texturedir", "").strip()
    
    # Find mesh files
    mesh_files = []
    for mesh_elem in root.findall(".//mesh[@file]"):
        file_attr = mesh_elem.get("file", "").strip()
        if file_attr:
            mesh_files.append(file_attr)
    
    # Find texture files
    texture_files = []
    for texture_elem in root.findall(".//texture[@file]"):
        file_attr = texture_elem.get("file", "").strip()
        if file_attr:
            texture_files.append(file_attr)
    
    return meshdir, texturedir, mesh_files, texture_files


def _resolve_asset_path(
    xml_base_dir: Optional[Path], asset_dir: Optional[str], filename: str
) -> Optional[Path]:
    """Resolve full path to asset file."""
    if not xml_base_dir:
        return None
    
    # Try with asset_dir first
    if asset_dir:
        clean_dir = asset_dir.strip("./\\")
        asset_path = xml_base_dir / clean_dir / filename
        if asset_path.exists():
            return asset_path
    
    # Try direct relative to XML
    fallback_path = xml_base_dir / filename
    if fallback_path.exists():
        return fallback_path
    
    return None


def _copy_external_assets(
    xml_content: str, xml_base_dir: Optional[Path], output_dir: Path
) -> tuple[list[str], list[str]]:
    """Copy external assets by parsing XML content."""
    print("Copying external assets from XML...")
    
    meshdir, texturedir, mesh_files_xml, texture_files_xml = _parse_xml_assets(xml_content)
    
    mesh_files = []
    texture_files = []
    
    # Copy meshes
    if mesh_files_xml:
        meshes_output_dir = output_dir / "meshes"
        meshes_output_dir.mkdir(exist_ok=True)
        
        print(f"  Found {len(mesh_files_xml)} mesh files")
        for mesh_filename in mesh_files_xml:
            source_path = _resolve_asset_path(xml_base_dir, meshdir, mesh_filename)
            
            if source_path and source_path.exists():
                destination = meshes_output_dir / source_path.name
                shutil.copy2(source_path, destination)
                mesh_files.append(f"meshes/{source_path.name}")
                print(f"  ✓ Copied mesh: {source_path.name}")
            else:
                print(f"  ✗ Mesh not found: {mesh_filename}")
    
    # Copy textures
    if texture_files_xml:
        textures_output_dir = output_dir / "textures"
        textures_output_dir.mkdir(exist_ok=True)
        
        print(f"  Found {len(texture_files_xml)} texture files")
        for texture_filename in texture_files_xml:
            source_path = _resolve_asset_path(xml_base_dir, texturedir, texture_filename)
            
            if source_path and source_path.exists():
                destination = textures_output_dir / source_path.name
                shutil.copy2(source_path, destination)
                texture_files.append(f"textures/{source_path.name}")
                print(f"  ✓ Copied texture: {source_path.name}")
            else:
                print(f"  ✗ Texture not found: {texture_filename}")
    
    if not mesh_files and not texture_files:
        print("  No external assets found")
    
    return mesh_files, texture_files


def _rewrite_xml_paths(
    xml_content: str, mesh_files: list[str], texture_files: list[str]
) -> str:
    """Rewrite asset paths in XML to use local relative paths."""
    try:
        root = ET.fromstring(xml_content)
        
        # Update compiler element
        compiler = root.find("compiler")
        if compiler is not None:
            if mesh_files:
                compiler.set("meshdir", "meshes/")
            if texture_files:
                compiler.set("texturedir", "textures/")
            elif "texturedir" in compiler.attrib:
                del compiler.attrib["texturedir"]
        
        # Convert back to string
        import xml.dom.minidom
        updated_xml = ET.tostring(root, encoding="unicode")
        dom = xml.dom.minidom.parseString(updated_xml)
        pretty_xml = dom.toprettyxml(indent="  ")
        # Remove extra blank lines
        return "\n".join([line for line in pretty_xml.split("\n") if line.strip()])
        
    except Exception:
        return xml_content
