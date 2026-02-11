"""Genesis + Browser: Franka arm pushing cube through water."""

import asyncio
import io
import time

import genesis as gs
import numpy as np
import trimesh

from viewport.genesis import extract, glb
from viewport.genesis.glb import Body
from viewport.visualizer import Visualizer

# Initialize Genesis
gs.init()

# Create scene
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

# Add ground
scene.add_entity(
    morph=gs.morphs.Plane(),
    material=gs.materials.Rigid(needs_coup=True),
)

# Add Franka arm
franka = scene.add_entity(
    morph=gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
    material=gs.materials.Rigid(needs_coup=True, coup_friction=0.8),
)

# Add cube
cube = scene.add_entity(
    morph=gs.morphs.Box(
        size=(0.04, 0.04, 0.04),
        pos=(0.65, 0.0, 0.02),
    ),
    material=gs.materials.Rigid(needs_coup=True),
)

# Add water pool
water = scene.add_entity(
    material=gs.materials.MPM.Liquid(rho=1000.0, E=5e4, nu=0.2),
    morph=gs.morphs.Box(pos=(0.65, 0.0, 0.1), size=(0.35, 0.35, 0.12)),
    surface=gs.surfaces.Rough(color=(0.2, 0.6, 0.95, 0.8), vis_mode="particle"),
)

scene.build()

# Extract GLB meshes for Franka
print(f"Extracting {len(franka.vgeoms)} visual geometries...")
bodies = glb.extract(franka)

# Add cube as a body
cube_mesh = trimesh.creation.box(extents=(0.08, 0.08, 0.08))
cube_mesh.visual.vertex_colors = [200, 100, 50, 255]

# Export cube to GLB
with io.BytesIO() as buf:
    cube_mesh.export(buf, file_type="glb")
    cube_glb = buf.getvalue()

cube_body = Body(
    id=len(bodies),
    name="cube",
    glb=cube_glb,
    fixed=False,
)
bodies.append(cube_body)

print(f"Extracted {len(bodies)} bodies (including cube)")

# Setup visualizer
vis = Visualizer(bodies, port=8080)  # type: ignore
vis.open()

# Get motor DOFs
motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

# Initial position
qpos = np.array([-1.0124, 1.5559, 1.3662, -1.6878, -1.5799, 1.7757, 1.4602, 0.04, 0.04])
franka.set_qpos(qpos)
scene.step()

end_effector = franka.get_link("hand")

# Motion phases
phase = 0
phase_step = 0
target_qpos = qpos.copy()

print("Running simulation (Ctrl+C to stop)...")
print("Watch the browser visualization...")

try:
    while True:
        # Step physics
        scene.step()

        # Motion control
        if phase == 0:  # Reach to grasp position
            target_qpos = franka.inverse_kinematics(
                link=end_effector,
                pos=np.array([0.65, 0.0, 0.135]),
                quat=np.array([0, 1, 0, 0]),
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.04, 0.04]), fingers_dof)

            phase_step += 1
            if phase_step > 100:
                phase = 1
                phase_step = 0
                print("Phase: Grasp")

        elif phase == 1:  # Grasp cube
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)

            phase_step += 1
            if phase_step > 50:
                phase = 2
                phase_step = 0
                print("Phase: Lift")

        elif phase == 2:  # Lift cube
            target_qpos = franka.inverse_kinematics(
                link=end_effector,
                pos=np.array([0.65, 0.0, 0.35]),
                quat=np.array([0, 1, 0, 0]),
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)

            phase_step += 1
            if phase_step > 150:
                phase = 3
                phase_step = 0
                print("Phase: Plunge")

        elif phase == 3:  # Plunge
            progress = min(phase_step / 200.0, 1.0)
            z = 0.35 - progress * 0.25

            target_qpos = franka.inverse_kinematics(
                link=end_effector,
                pos=np.array([0.65, 0.0, z]),
                quat=np.array([0, 1, 0, 0]),
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)

            phase_step += 1

        # Extract Franka transforms
        positions, quaternions = extract.transforms(scene, franka)

        # Get cube position and quaternion
        cube_pos = cube.get_pos()
        cube_quat = cube.get_quat()

        if hasattr(cube_pos, "cpu"):
            cube_pos = cube_pos.cpu().numpy()
        if hasattr(cube_quat, "cpu"):
            cube_quat = cube_quat.cpu().numpy()

        # Append cube to transforms (same format as MuJoCo: all positions, then all quaternions)
        all_positions = np.vstack([positions, cube_pos.reshape(1, 3)])
        all_quaternions = np.vstack([quaternions, cube_quat.reshape(1, 4)])

        # Get water particles
        particle_pos = extract.particles(water)

        # Combine: transforms + particles
        data_parts = [
            all_positions.flatten(),
            all_quaternions.flatten(),
        ]

        # Append particle positions if any
        if len(particle_pos) > 0:
            data_parts.append(particle_pos.flatten())

        combined_data = np.concatenate(data_parts).astype(np.float32)

        # Schedule broadcast on server's event loop
        assert vis.server is not None and vis._loop is not None
        asyncio.run_coroutine_threadsafe(
            vis.server.broadcast(combined_data.tobytes()), vis._loop
        )

        # Maintain timing
        time.sleep(scene.sim_options.dt)

except KeyboardInterrupt:
    vis.close()
    print("\nStopped")
