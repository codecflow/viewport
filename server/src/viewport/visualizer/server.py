"""HTTP and WebSocket server for streaming body GLBs and transforms."""

import uvicorn
from starlette.applications import Starlette
from starlette.middleware import Middleware
from starlette.middleware.cors import CORSMiddleware
from starlette.responses import JSONResponse, Response
from starlette.routing import Route, WebSocketRoute
from starlette.websockets import WebSocket

from ..mujoco.glb import Body


class VisualizerServer:
    """HTTP + WebSocket server for streaming GLB meshes and transforms."""

    def __init__(self, bodies: list[Body], port: int = 8080):
        self.bodies = bodies
        self.port = port
        self.clients: set[WebSocket] = set()

        self.app = Starlette(
            middleware=[
                Middleware(
                    CORSMiddleware,
                    allow_origins=["*"],
                    allow_methods=["GET"],
                    allow_headers=["*"],
                )
            ],
            routes=[
                Route("/bodies", self.bodies_manifest),
                Route("/body/{body_id:int}.glb", self.serve_glb),
                WebSocketRoute("/ws", self.websocket_endpoint),
            ],
        )

    async def bodies_manifest(self, request):
        manifest = [
            {
                "id": int(b.id),
                "name": b.name,
                "fixed": bool(b.fixed),
                "size": len(b.glb),
            }
            for b in self.bodies
        ]
        return JSONResponse(manifest)

    async def serve_glb(self, request):
        body_id = int(request.path_params["body_id"])
        for b in self.bodies:
            if b.id == body_id:
                return Response(
                    content=b.glb,
                    media_type="model/gltf-binary",
                    headers={"Content-Disposition": f'inline; filename="{b.name}.glb"'},
                )
        return Response(status_code=404)

    async def websocket_endpoint(self, websocket: WebSocket):
        await websocket.accept()
        self.clients.add(websocket)
        try:
            while True:
                await websocket.receive_bytes()
        except Exception:
            pass
        finally:
            self.clients.discard(websocket)

    async def broadcast(self, data: bytes) -> None:
        for client in list(self.clients):
            try:
                await client.send_bytes(data)
            except Exception:
                self.clients.discard(client)

    async def start(self):
        config = uvicorn.Config(
            self.app, host="0.0.0.0", port=self.port, log_level="error"
        )
        server = uvicorn.Server(config)
        await server.serve()
