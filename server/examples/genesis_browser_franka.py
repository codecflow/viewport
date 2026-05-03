"""Genesis + Browser: Franka arm pushing cube through water."""

import asyncio
import threading
import time
import webbrowser

import genesis as gs
import numpy as np

from viewport.genesis import extract, glb as gglb, scene as gscene
from viewport.visualizer.server import VisualizerServer

# Initialize Genesis
gs.init()

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.01, substeps=10),
    rigid_options=gs.options.RigidOptions(
        max_collision_pairs=500,
        box_box_detection=True,
    ),
    mpm_options=gs.options.MPMOptions(
        lower_bound=(0.2, -0.4, -0.1),
        upper_bound=(1.2, 0.4, 0.8),
        grid_density=64,
    ),
    renderer=None,
    show_viewer=False,
)

scene.add_entity(
    morph=gs.morphs.Plane(),
    material=gs.materials.Rigid(needs_coup=True),
)

franka = scene.add_entity(
    morph=gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
    material=gs.materials.Rigid(needs_coup=True, coup_friction=0.8),
)

cube = scene.add_entity(
    morph=gs.morphs.Box(size=(0.04, 0.04, 0.04), pos=(0.65, 0.0, 0.02)),
    material=gs.materials.Rigid(needs_coup=True),
)

water = scene.add_entity(
    material=gs.materials.MPM.Liquid(rho=1000.0, E=5e4, nu=0.2),
    morph=gs.morphs.Box(pos=(0.65, 0.0, 0.1), size=(0.35, 0.35, 0.12)),
    surface=gs.surfaces.Rough(color=(0.2, 0.6, 0.95, 0.8), vis_mode="particle"),
)

scene.build()

# ── Build VisualScene + server ──────────────────────────────────────────────────

PORT = 8080
api_base = f"http://localhost:{PORT}"

print(f"Extracting {len(franka.vgeoms)} visual geometries...")
glb_bodies = gglb.extract(franka)

# Extract full VisualScene: franka (articulated), cube (rigid box), water (particles)
visual = gscene.extract(
    [franka, cube, water],
    glb_bodies_map={franka: glb_bodies},
    base_url=api_base,
)

# Manifest: body order matches transform stream order (franka vgeoms, then cube)
rigid_bodies = visual["bodies"]
nbody = len(rigid_bodies)
manifest = [{"id": i, "name": b["name"]} for i, b in enumerate(rigid_bodies)]

server = VisualizerServer(visual, glb_bodies, nbody, PORT, manifest=manifest)

loop = asyncio.new_event_loop()
thread = threading.Thread(target=lambda: loop.run_until_complete(server.start()), daemon=True)
thread.start()
time.sleep(0.5)

ws_url = f"ws://localhost:{PORT}/ws"
url = f"http://localhost:5200/?ws={ws_url}"
print(f"🌐 Opening {url}")
webbrowser.open(url)

# ── Simulation ──────────────────────────────────────────────────────────────────

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)
qpos = np.array([-1.0124, 1.5559, 1.3662, -1.6878, -1.5799, 1.7757, 1.4602, 0.04, 0.04])
franka.set_qpos(qpos)
scene.step()

end_effector = franka.get_link("hand")
phase = 0
phase_step = 0
target_qpos = qpos.copy()

print("Running simulation (Ctrl+C to stop)...")

try:
    while True:
        scene.step()

        if phase == 0:
            target_qpos = franka.inverse_kinematics(
                link=end_effector, pos=np.array([0.65, 0.0, 0.135]), quat=np.array([0, 1, 0, 0])
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.04, 0.04]), fingers_dof)
            phase_step += 1
            if phase_step > 100:
                phase, phase_step = 1, 0
                print("Phase: Grasp")

        elif phase == 1:
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)
            phase_step += 1
            if phase_step > 50:
                phase, phase_step = 2, 0
                print("Phase: Lift")

        elif phase == 2:
            target_qpos = franka.inverse_kinematics(
                link=end_effector, pos=np.array([0.65, 0.0, 0.35]), quat=np.array([0, 1, 0, 0])
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)
            phase_step += 1
            if phase_step > 150:
                phase, phase_step = 3, 0
                print("Phase: Plunge")

        elif phase == 3:
            z = 0.35 - min(phase_step / 200.0, 1.0) * 0.25
            target_qpos = franka.inverse_kinematics(
                link=end_effector, pos=np.array([0.65, 0.0, z]), quat=np.array([0, 1, 0, 0])
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)
            phase_step += 1

        # Build transform stream: [franka_pos | cube_pos | franka_quat | cube_quat]
        positions, quaternions = extract.transforms(scene, franka)
        cube_pos = cube.get_pos()
        cube_quat = cube.get_quat()
        if hasattr(cube_pos, "cpu"): cube_pos = cube_pos.cpu().numpy()
        if hasattr(cube_quat, "cpu"): cube_quat = cube_quat.cpu().numpy()

        all_positions = np.vstack([positions, cube_pos.reshape(1, 3)])
        all_quaternions = np.vstack([quaternions, cube_quat.reshape(1, 4)])

        particle_pos = extract.particles(water)
        parts = [all_positions.flatten(), all_quaternions.flatten()]
        if len(particle_pos) > 0:
            parts.append(particle_pos.flatten())

        data = np.concatenate(parts).astype(np.float32)
        asyncio.run_coroutine_threadsafe(server.broadcast(data.tobytes()), loop)

        time.sleep(scene.sim_options.dt)

except KeyboardInterrupt:
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=1.0)
    print("\nStopped")
