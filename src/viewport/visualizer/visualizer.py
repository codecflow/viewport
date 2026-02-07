"""Main Visualizer class for streaming MuJoCo simulations."""

import asyncio
import webbrowser
from pathlib import Path

import mujoco
import numpy as np

from viewport.mujoco.glb import Body
from .server import VisualizerServer


class Visualizer:
    """Visualize MuJoCo simulation in browser with Three.js.
    
    Usage:
        bodies = glb.extract(model)
        vis = Visualizer(bodies)
        vis.open()  # Opens browser
        
        while True:
            mujoco.mj_step(model, data)
            vis.update(data)
    """
    
    def __init__(self, bodies: list[Body], port: int = 8080):
        """Initialize visualizer.
        
        Args:
            bodies: List of Body with GLB data from glb.extract()
            port: HTTP server port
        """
        self.bodies = bodies
        self.port = port
        self.server: VisualizerServer | None = None
        self._task: asyncio.Task | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
    
    def open(self) -> None:
        """Start server and open browser."""
        static_dir = Path(__file__).parent / "static"
        self.server = VisualizerServer(self.bodies, static_dir, self.port)
        
        # Start server in background
        self._loop = asyncio.new_event_loop()
        self._task = self._loop.create_task(self.server.start())
        
        # Run briefly to let server start
        self._loop.run_until_complete(asyncio.sleep(0.5))
        
        # Open browser
        url = f"http://localhost:{self.port}"
        print(f"🌐 Visualizer running at {url}")
        webbrowser.open(url)
    
    def update(self, data: mujoco.MjData) -> None:
        """Send transform update to browser.
        
        Args:
            data: MuJoCo data with current state
        """
        if not self.server:
            return
        
        # Extract xpos and xquat for all bodies
        # data.xpos: (nbody, 3), data.xquat: (nbody, 4) in wxyz format
        transforms = np.concatenate([
            data.xpos.flatten(),  # All positions
            data.xquat.flatten()  # All quaternions (wxyz)
        ]).astype(np.float32)
        
        # Send via WebSocket
        self.server.broadcast(transforms.tobytes())
        
        # Process events briefly
        if self._loop:
            self._loop.run_until_complete(asyncio.sleep(0.001))
    
    def close(self) -> None:
        """Stop server and cleanup."""
        if self._task:
            self._task.cancel()
        if self._loop:
            self._loop.close()
        self.server = None
