"""Main Visualizer class for streaming MuJoCo simulations."""

import asyncio
import threading
import time
import webbrowser

import mujoco
import numpy as np

from ..mujoco import glb, scene as mjscene
from .server import VisualizerServer


class Visualizer:
    """Visualize MuJoCo simulation in browser with Three.js.

    Starts a local HTTP/WebSocket server that streams VisualScene + live transforms.
    Open the preview app at ?ws=ws://localhost:{port}/ws to render.

    Usage:
        vis = Visualizer(model)
        vis.open("http://localhost:5200")

        while True:
            mujoco.mj_step(model, data)
            vis.update(data)
    """

    def __init__(self, model: mujoco.MjModel, port: int = 8080):
        self.model = model
        self.port = port
        self.server: VisualizerServer | None = None
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def open(self, viewer_url: str | None = None) -> None:
        """Start the API server and open a viewer."""
        api_base = f"http://localhost:{self.port}"
        bodies = glb.extract(self.model)
        visual = mjscene.extract(self.model, base_url=api_base)
        self.server = VisualizerServer(visual, bodies, self.model.nbody, self.port)

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        time.sleep(0.5)

        ws_url = f"ws://localhost:{self.port}/ws"

        if viewer_url:
            url = f"{viewer_url.rstrip('/')}/?ws={ws_url}"
            print(f"🌐 Opening {viewer_url} (API on {api_base})")
            webbrowser.open(url)
        else:
            print(f"🌐 API server running at {api_base}")
            print(f"   Connect a viewer with: ?ws={ws_url}")

    def _run_server(self) -> None:
        assert self.server is not None
        assert self._loop is not None
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self.server.start())
        except RuntimeError:
            pass

    def update(self, data: mujoco.MjData) -> None:
        """Send transform update to browser."""
        if not self.server or not self._loop:
            return

        transforms = np.concatenate(
            [data.xpos.flatten(), data.xquat.flatten()]
        ).astype(np.float32)

        asyncio.run_coroutine_threadsafe(
            self.server.broadcast(transforms.tobytes()), self._loop
        )

    def close(self) -> None:
        """Stop server and cleanup."""
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=1.0)
        self.server = None
