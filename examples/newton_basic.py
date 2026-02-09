"""Basic Newton example using built-in ViewerRerun."""

import warp as wp
import newton

# Initialize Warp
wp.init()

# Build scene
builder = newton.ModelBuilder()

# Add ground
builder.add_ground_plane()

# Add falling sphere
sphere_pos = wp.vec3(0.0, 0.0, 2.0)
body_sphere = builder.add_body(xform=wp.transform(sphere_pos, wp.quat_identity()), key="sphere")
builder.add_shape_sphere(body_sphere, radius=0.5)

# Add falling box
box_pos = wp.vec3(1.0, 0.0, 3.0)
body_box = builder.add_body(xform=wp.transform(box_pos, wp.quat_identity()), key="box")
builder.add_shape_box(body_box, hx=0.3, hy=0.3, hz=0.3)

# Add falling capsule
capsule_pos = wp.vec3(-1.0, 0.0, 2.5)
body_capsule = builder.add_body(xform=wp.transform(capsule_pos, wp.quat_identity()), key="capsule")
builder.add_shape_capsule(body_capsule, radius=0.2, half_height=0.5)

# Finalize model
model = builder.finalize()

# Create solver
solver = newton.solvers.SolverXPBD(model, iterations=10)

# Create states
state_0 = model.state()
state_1 = model.state()
control = model.control()

# Create Newton's built-in Rerun 
viewer = newton.viewer.ViewerRerun(app_id="newton-basic")
viewer.set_model(model)

# Simulation parameters
fps = 60
frame_dt = 1.0 / fps
sim_substeps = 10
sim_dt = frame_dt / sim_substeps

print("Newton simulation starting...")
print(f"Bodies: {model.body_count}")
print(f"Shapes: {model.shape_count}")
print(f"Running at {fps} FPS with {sim_substeps} substeps")
print("Press Ctrl+C to stop")

# Simulation loop
frame = 0
sim_time = 0.0

try:
    while viewer.is_running():
        # Begin frame
        viewer.begin_frame(sim_time)
        
        # Substep physics
        for _ in range(sim_substeps):
            state_0.clear_forces()
            contacts = model.collide(state_0)
            solver.step(state_0, state_1, control, contacts, sim_dt)
            state_0, state_1 = state_1, state_0
        
        sim_time += frame_dt
        
        # Log state to viewer (handles all mesh + transform updates)
        viewer.log_state(state_0)
        
        # End frame
        viewer.end_frame()
        
        frame += 1
        
        if frame % 100 == 0:
            print(f"Frame {frame}, time: {sim_time:.2f}s")

except KeyboardInterrupt:
    print("\nStopped")
finally:
    viewer.close()
