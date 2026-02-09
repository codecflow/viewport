"""Genesis + Rerun: Franka arm pushing cube through water."""

import genesis as gs
import numpy as np
import rerun as rr

from viewport.genesis import extract

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

# Add cube to push
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
    surface=gs.surfaces.Rough(color=(0.2, 0.6, 0.95, 0.8), vis_mode='particle'),
)

scene.build()

# Initialize Rerun
rr.init("franka-water", spawn=True)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

# Extract and log Franka meshes (static)
print(f"Extracting {len(franka.vgeoms)} visual geometries...")
mesh_data = extract.meshes(franka)

for name, mesh in mesh_data:
    vertices = np.array(mesh.vertices, dtype=np.float32)
    faces = np.array(mesh.faces, dtype=np.uint32)
    
    colors = None
    if hasattr(mesh.visual, "vertex_colors"):
        colors = np.array(mesh.visual.vertex_colors, dtype=np.uint8)
    
    rr.log(
        f"world/franka/{name}",
        rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=faces,
            vertex_colors=colors,
        ),
        static=True,
    )

print(f"Logged {len(mesh_data)} meshes to Rerun")

# Get motor DOFs
motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

# Initial position
qpos = np.array([-1.0124, 1.5559, 1.3662, -1.6878, -1.5799, 1.7757, 1.4602, 0.04, 0.04])
franka.set_qpos(qpos)
scene.step()

end_effector = franka.get_link("hand")

# Motion sequence
print("Starting simulation...")
frame = 0
sim_time = 0.0
phase = 0  # 0: reach, 1: grasp, 2: lift, 3: plunge
phase_step = 0

# Store target qpos
target_qpos = qpos.copy()

try:
    for step in range(5000):
        scene.step()
        sim_time += scene.sim_options.dt
        
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
                print("Phase: Plunge into water")
        
        elif phase == 3:  # Plunge into water
            progress = min(phase_step / 200.0, 1.0)
            z = 0.35 - progress * 0.25  # Lower from 0.35 to 0.10
            
            target_qpos = franka.inverse_kinematics(
                link=end_effector,
                pos=np.array([0.65, 0.0, z]),
                quat=np.array([0, 1, 0, 0]),
            )
            franka.control_dofs_position(target_qpos[:-2], motors_dof)
            franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)
            
            phase_step += 1
        
        # Extract and log transforms
        positions, quaternions = extract.transforms(scene, franka)
        
        rr.set_time("frame", sequence=frame)
        rr.set_time("sim_time", duration=sim_time)
        
        # Log Franka transforms
        for i, (name, _) in enumerate(mesh_data):
            pos = positions[i]
            quat_wxyz = quaternions[i]
            quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
            
            rr.log(
                f"world/franka/{name}",
                rr.Transform3D(
                    translation=pos,
                    quaternion=quat_xyzw,
                ),
            )
        
        # Log cube position
        cube_pos = cube.get_pos()
        if hasattr(cube_pos, 'cpu'):
            cube_pos = cube_pos.cpu().numpy()
        rr.log(
            "world/cube",
            rr.Boxes3D(
                half_sizes=[0.02, 0.02, 0.02],
                centers=cube_pos,
                colors=[200, 100, 50],
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
            print(f"Step {step}, time: {sim_time:.2f}s, phase: {phase}")

except KeyboardInterrupt:
    print("\nStopped")
