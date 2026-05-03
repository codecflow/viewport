"""Newton simulation visualized in the browser via VisualizerServer."""

import asyncio
import threading
import time
import webbrowser

import newton
import numpy as np
import warp as wp

from viewport.visualizer.server import VisualizerServer

# Body shape colors (r,g,b,a 0-1)
_COLORS = [
    [1.0, 0.35, 0.35, 1.0],  # red
    [0.35, 1.0, 0.45, 1.0],  # green
    [0.35, 0.55, 1.0, 1.0],  # blue
    [1.0, 0.85, 0.2, 1.0],   # yellow
]

wp.init()

# ── Build scene ───────────────────────────────────────────────────────────────

builder = newton.ModelBuilder()
builder.add_ground_plane()

body_sphere = builder.add_body(
    xform=wp.transform(wp.vec3(0.0, 0.0, 2.0), wp.quat_identity(dtype=float)),
    label="sphere",
)
builder.add_shape_sphere(body_sphere, radius=0.5)

body_box = builder.add_body(
    xform=wp.transform(wp.vec3(1.0, 0.0, 3.0), wp.quat_identity(dtype=float)),
    label="box",
)
builder.add_shape_box(body_box, hx=0.3, hy=0.3, hz=0.3)

body_capsule = builder.add_body(
    xform=wp.transform(wp.vec3(-1.0, 0.0, 2.5), wp.quat_identity(dtype=float)),
    label="capsule",
)
builder.add_shape_capsule(body_capsule, radius=0.2, half_height=0.5)

model = builder.finalize()
solver = newton.solvers.SolverXPBD(model, iterations=10)
state_0 = model.state()
state_1 = model.state()
control = model.control()

# ── Extract VisualScene ───────────────────────────────────────────────────────

_GEO = newton.GeoType  # PLANE=1, SPHERE=3, CAPSULE=4, CYLINDER=6, BOX=7


def _geom_for_shape(sid: int, shape_type, shape_scale, color: list) -> dict | None:
    t = int(shape_type[sid])
    s = shape_scale[sid]
    mat = {"color": color}
    if t == int(_GEO.SPHERE):
        return {"type": "sphere", "size": [round(float(s[0]), 6)], "material": mat}
    if t == int(_GEO.BOX):
        return {"type": "box", "size": [round(float(s[i] * 2), 6) for i in range(3)], "material": mat}
    if t == int(_GEO.CAPSULE):
        return {"type": "capsule", "size": [round(float(s[0]), 6), round(float(s[1] * 2), 6)], "material": mat}
    if t == int(_GEO.CYLINDER):
        return {"type": "cylinder", "size": [round(float(s[0]), 6), round(float(s[1] * 2), 6)], "material": mat}
    return None


def _build_visual_scene(model) -> dict:
    # body_q: (nbody, 7) = [px, py, pz, qx, qy, qz, qw] (warp xyzw)
    body_q = model.body_q.numpy().reshape(-1, 7)
    shape_type = model.shape_type.numpy()
    shape_scale = model.shape_scale.numpy().reshape(-1, 3)
    shape_body = model.shape_body.numpy()

    bodies = []

    # Ground plane (shape_body == -1, not a tracked body)
    bodies.append({
        "kind": "rigid",
        "name": "ground",
        "pos": [0.0, 0.0, 0.0],
        "geoms": [{"type": "plane", "size": [10.0, 10.0], "material": {"color": [0.85, 0.85, 0.85, 1.0]}}],
    })

    # Dynamic bodies
    for bid in range(model.body_count):
        name = model.body_label[bid] if bid < len(model.body_label) else f"body_{bid}"
        q = body_q[bid]
        pos = [round(float(x), 6) for x in q[:3]]
        # xyzw → wxyz
        quat = [round(float(q[6]), 6), round(float(q[3]), 6), round(float(q[4]), 6), round(float(q[5]), 6)]

        color = _COLORS[bid % len(_COLORS)]
        geoms = []
        for sid in range(model.shape_count):
            if int(shape_body[sid]) != bid:
                continue
            g = _geom_for_shape(sid, shape_type, shape_scale, color)
            if g:
                geoms.append(g)

        if not geoms:
            geoms = [{"type": "sphere", "size": [0.1], "material": {"color": [0.6, 0.6, 0.6, 1.0]}}]

        body: dict = {"kind": "rigid", "name": name, "pos": pos, "geoms": geoms}
        identity = abs(quat[0] - 1.0) < 1e-5
        if not identity:
            body["quat"] = quat
        bodies.append(body)

    return {
        "bodies": bodies,
        "lights": [{"type": "directional", "name": "sun", "pos": [5.0, 5.0, 10.0], "dir": [0.0, 0.0, -1.0], "intensity": 1.0}],
        "cameras": [], "sensors": [],
    }


def _get_transforms(state) -> bytes:
    """Pack body transforms as [nbody*3 positions | nbody*4 quats_wxyz] Float32."""
    body_q = state.body_q.numpy().reshape(-1, 7)
    positions = body_q[:, :3]                                             # (nbody, 3)
    q_xyzw = body_q[:, 3:7]                                               # (nbody, 4) xyzw
    q_wxyz = np.hstack([q_xyzw[:, 3:4], q_xyzw[:, :3]])                  # (nbody, 4) wxyz
    return np.concatenate([positions.flatten(), q_wxyz.flatten()]).astype(np.float32).tobytes()


# ── Server setup ──────────────────────────────────────────────────────────────

PORT = 8080
visual = _build_visual_scene(model)

# Manifest: only dynamic bodies (ground is not in transform stream)
nbody = model.body_count
manifest = [
    {"id": i, "name": model.body_label[i] if i < len(model.body_label) else f"body_{i}"}
    for i in range(nbody)
]

server = VisualizerServer(visual, [], nbody, PORT, manifest=manifest)

loop = asyncio.new_event_loop()
thread = threading.Thread(target=lambda: loop.run_until_complete(server.start()), daemon=True)
thread.start()
time.sleep(0.5)

ws_url = f"ws://localhost:{PORT}/ws"
url = f"http://localhost:5200/?ws={ws_url}"
print(f"🌐 Opening {url}")
print(f"Bodies: {model.body_count} | Shapes: {model.shape_count}")
webbrowser.open(url)

# ── Simulation loop ───────────────────────────────────────────────────────────

fps = 60
frame_dt = 1.0 / fps
sim_substeps = 10
sim_dt = frame_dt / sim_substeps
frame = 0
sim_time = 0.0

print("Running (Ctrl+C to stop)...")

try:
    while True:
        for _ in range(sim_substeps):
            state_0.clear_forces()
            contacts = model.collide(state_0)
            solver.step(state_0, state_1, control, contacts, sim_dt)
            state_0, state_1 = state_1, state_0

        sim_time += frame_dt
        asyncio.run_coroutine_threadsafe(server.broadcast(_get_transforms(state_0)), loop)
        time.sleep(frame_dt)

        frame += 1
        if frame % 100 == 0:
            print(f"Frame {frame}, t={sim_time:.2f}s")

except KeyboardInterrupt:
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=1.0)
    print("\nStopped")
