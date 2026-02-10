"""Export stacking scene to static WASM site."""

from pathlib import Path
import mujoco
from viewport.mujoco import wasm

# Load model
scene_path = Path(__file__).parent / "scenes/stacking.xml"
model = mujoco.MjModel.from_xml_path(str(scene_path))

# Export to static WASM site
output = Path(__file__).parent / "dist" / "stacking"
wasm.export(
    model=model,
    output=output,
    title="Stacking - MuJoCo WASM",
    xml_path=scene_path,
)

print(f"\n✓ Ready! Open {output / 'index.html'} in a browser")
print("  Or run: python -m http.server -d examples/dist/stacking 8000")
