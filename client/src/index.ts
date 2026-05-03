export { createViewport } from './viewport.js';
export type { Viewport, ViewportOpts } from './viewport.js';

export { buildBodyIdx, applyTransforms, flattenBodies, restoreBodies } from './sync.js';
export type { BodySnapshot } from './sync.js';
export { loadBodies, connectSim } from './sync.js';
export type { BodyInfo, LoadBodiesOpts, LoadBodiesResult, SimOpts, SimHandle } from './sync.js';

export type { SimScene, SimFrame, Contact, SimConnection, SimCommand, SimEvent } from './types.js';

export { LidarViz } from './sensors/index.js';
export type { LidarScanConfig, ScanPattern } from './sensors/index.js';
export { buildDirs, dirs2D, dirs3D, dirsLivox, dirsFlash } from './sensors/index.js';
export { ContactViz } from './sensors/index.js';

export type {
	GeomDesc, BodyDesc, RigidBodyDesc, SoftBodyDesc,
	SensorDesc, SimCamera, VisualScene,
	MaterialDesc, LightDesc, LightType,
	ParticleDesc, EnvironmentDesc
} from './viewer/types.js';

export { SimScene as SimRenderer } from './viewer/scene.js';
export type { VizState } from './viewer/scene.js';

export { buildSceneFromVisual, buildGeomMesh, createPrimitiveMesh, createLightFromDesc } from './viewer/builder.js';
export type { BuiltScene } from './viewer/builder.js';
