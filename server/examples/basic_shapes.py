import time
from pathlib import Path

import mujoco

from viewport.visualizer import Visualizer


def main():
    scene_path = Path(__file__).parent / "scenes/basic_shapes.xml"

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    vis = Visualizer(model, port=8080)
    vis.open("http://localhost:5200")

    print("Running simulation (Ctrl+C to stop)...")

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
