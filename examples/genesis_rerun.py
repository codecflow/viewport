"""Genesis + Rerun visualization example."""

import genesis as gs
import numpy as np
import rerun as rr

from viewport.genesis import extract

# Initialize Genesis
gs.init()

# Create scene
scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.01, substeps=10),
    rigid_options=gs.options.RigidOptions(max_collision_pairs=500),
    mpm_options=gs.options.MPMOptions(
        lower_bound=(-1, -1, -0.1),
        upper_bound=(1, 1, 1.5),
        grid_density=32,
    ),
    renderer=None,
    show_viewer=False,
)

# Add ground with MPM coupling
scene.add_entity(
    morph=gs.morphs.Plane(),
    material=gs.materials.Rigid(needs_coup=True, coup_friction=0.5),
)

# Add robot (Go2 quadruped - bundled with genesis-world)
robot = scene.add_entity(
    morph=gs.morphs.URDF(
        file="urdf/go2/urdf/go2.urdf",
        pos=(0, 0, 0.4),
    ),
    material=gs.materials.Rigid(
        needs_coup=True, coup_friction=0.8, coup_softness=0.0005
    ),
)

# Add water cube falling above robot
water = scene.add_entity(
    material=gs.materials.MPM.Liquid(rho=1000.0, E=5e4, nu=0.2),
    morph=gs.morphs.Box(pos=(0, 0, 0.9), size=(0.3, 0.3, 0.3)),
    surface=gs.surfaces.Rough(color=(0.2, 0.6, 0.95, 0.8), vis_mode="particle"),
)

scene.build()

# Initialize Rerun
rr.init("genesis-sim", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

# Extract and log meshes (static)
print(f"Extracting {len(robot.vgeoms)} visual geometries...")
mesh_data = extract.meshes(robot)

for name, mesh in mesh_data:
    vertices = np.array(mesh.vertices, dtype=np.float32)
    faces = np.array(mesh.faces, dtype=np.uint32)

    # Extract colors if available
    colors = None
    if mesh.visual is not None and hasattr(mesh.visual, "vertex_colors"):
        colors = np.array(mesh.visual.vertex_colors, dtype=np.uint8)  # type: ignore

    # Log mesh
    rr.log(
        f"world/robot/{name}",
        rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=faces,
            vertex_colors=colors,
        ),
        static=True,
    )

print(f"Logged {len(mesh_data)} meshes to Rerun")

# Setup simple joint control
joint_names = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
]

motor_dofs = []
for name in joint_names:
    try:
        joint = robot.get_joint(name)
        if joint:
            motor_dofs.extend(joint.dofs_idx_local)
    except Exception:
        pass

if motor_dofs:
    import torch

    kp = [100.0] * len(motor_dofs)
    kv = [10.0] * len(motor_dofs)
    robot.set_dofs_kp(kp, motor_dofs)
    robot.set_dofs_kv(kv, motor_dofs)

    # Set initial standing position
    pos = torch.tensor([0.0, 0.67, -1.3] * 4, dtype=torch.float32)
    robot.control_dofs_position(pos, motor_dofs)

    print(f"Configured {len(motor_dofs)} motor DOFs")

# Simulation loop
print("Starting simulation... Press Ctrl+C to stop")

frame = 0
sim_time = 0.0

try:
    for step in range(10000):
        # Step physics
        scene.step()
        sim_time += scene.sim_options.dt

        # Simple walking animation
        if motor_dofs and len(motor_dofs) >= 12:
            import torch

            phase = sim_time * 2.0
            amp = 0.3

            target = torch.tensor(
                [
                    0.0,
                    0.67 + amp * np.sin(phase),
                    -1.3,
                    0.0,
                    0.67 + amp * np.sin(phase + np.pi),
                    -1.3,
                    0.0,
                    0.67 + amp * np.sin(phase + np.pi),
                    -1.3,
                    0.0,
                    0.67 + amp * np.sin(phase),
                    -1.3,
                ],
                dtype=torch.float32,
            )
            robot.control_dofs_position(target, motor_dofs)

        # Extract and log transforms
        positions, quaternions = extract.transforms(scene, robot)

        # Set timeline
        rr.set_time("frame", sequence=frame)
        rr.set_time("sim_time", duration=sim_time)

        # Log robot transforms (quaternion: wxyz → xyzw for Rerun)
        for i, (name, _) in enumerate(mesh_data):
            pos = positions[i]
            quat_wxyz = quaternions[i]
            quat_xyzw = np.array(
                [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
            )

            rr.log(
                f"world/robot/{name}",
                rr.Transform3D(
                    translation=pos,
                    quaternion=quat_xyzw,  # type: ignore
                ),
            )

        # Log water particles
        particle_pos = extract.particles(water)
        if len(particle_pos) > 0:
            rr.log(
                "world/particles",
                rr.Points3D(
                    positions=particle_pos,
                    colors=[51, 153, 242],
                    radii=0.005,
                ),
            )

        frame += 1

        if step % 100 == 0:
            print(f"Step {step}, time: {sim_time:.2f}s")

except KeyboardInterrupt:
    print("\nStopped")
