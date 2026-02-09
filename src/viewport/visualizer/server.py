"""HTTP and WebSocket server for visualizer."""

import asyncio
from pathlib import Path

from starlette.applications import Starlette
from starlette.responses import JSONResponse, Response
from starlette.routing import Mount, Route, WebSocketRoute
from starlette.staticfiles import StaticFiles
from starlette.websockets import WebSocket
import uvicorn

from ..mujoco.glb import Body


class VisualizerServer:
    """HTTP + WebSocket server for streaming GLB meshes and transforms."""
    
    def __init__(self, bodies: list[Body], static_dir: Path, port: int = 8080):
        self.bodies = bodies
        self.static_dir = static_dir
        self.port = port
        self.clients: set[WebSocket] = set()
        
        # Create Starlette app
        self.app = Starlette(
            routes=[
                Route("/bodies", self.bodies_manifest),
                Route("/body/{body_id:int}.glb", self.serve_glb),
                WebSocketRoute("/ws", self.websocket_endpoint),
                Mount("/", StaticFiles(directory=str(static_dir), html=True), name="static"),
            ]
        )
    
    async def bodies_manifest(self, request):
        """Return JSON manifest of all bodies."""
        manifest = [
            {
                "id": int(body.id),
                "name": body.name,
                "fixed": bool(body.fixed),
                "size": len(body.glb)
            }
            for body in self.bodies
        ]
        return JSONResponse(manifest)
    
    async def serve_glb(self, request):
        """Serve GLB file for a body."""
        body_id = int(request.path_params["body_id"])
        
        # Find body
        for body in self.bodies:
            if body.id == body_id:
                return Response(
                    content=body.glb,
                    media_type="model/gltf-binary",
                    headers={"Content-Disposition": f'inline; filename="{body.name}.glb"'}
                )
        
        return Response(status_code=404)
    
    async def websocket_endpoint(self, websocket: WebSocket):
        """Handle WebSocket connection for streaming transforms."""
        await websocket.accept()
        self.clients.add(websocket)
        
        try:
            # Keep connection alive
            while True:
                await websocket.receive_bytes()
        except Exception:
            pass
        finally:
            self.clients.discard(websocket)
    
    async def broadcast(self, data: bytes) -> None:
        """Broadcast transform data to all connected clients."""
        for client in list(self.clients):
            try:
                await client.send_bytes(data)
            except Exception:
                self.clients.discard(client)
    
    async def start(self):
        """Start the server."""
        config = uvicorn.Config(
            self.app,
            host="127.0.0.1",
            port=self.port,
            log_level="error",
        )
        server = uvicorn.Server(config)
        await server.serve()
