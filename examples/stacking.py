import time
from pathlib import Path

import mujoco
from viewport.mujoco import glb
from viewport.visualizer import Visualizer

def main():
    scene_path = Path(__file__).parent / "scenes/stacking.xml"
    
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    
    bodies = glb.extract(model)
    vis = Visualizer(bodies, port=8080)
    vis.open()
    
    print("Running box stacking (Ctrl+C to stop)...")
    print("Watch the boxes fall and stack!")
    
    try:
        while True:
            mujoco.mj_step(model, data)
            vis.update(data)
            time.sleep(model.opt.timestep)
    except KeyboardInterrupt:
        vis.close()
        print("\nStopped")

if __name__ == "__main__":
    main()
