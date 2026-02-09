"""Rerun visualizer example: falling and stacking boxes."""

import mujoco
from viewport.rerun import RerunVisualizer

model = mujoco.MjModel.from_xml_path("examples/scenes/stacking.xml")
data = mujoco.MjData(model)

vis = RerunVisualizer(model)
vis.open()

print("Rerun viewer opened. Running simulation...")
print("Press Ctrl+C to stop")

try:
    for step in range(10000):
        mujoco.mj_step(model, data)
        vis.update(data)
        
        if step % 100 == 0:
            print(f"Step {step}, time: {data.time:.2f}s")
            
except KeyboardInterrupt:
    print("\nStopped")
finally:
    vis.close()
