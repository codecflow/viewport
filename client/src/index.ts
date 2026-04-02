export { buildScene, createGeomMesh, buildLightComponent } from './compiler/threejs/index.js';
export type { ViewerScene } from './compiler/threejs/index.js';

export { parseThreeScene } from './parser/threejs/index.js';
export type { ThreeEntity, ParseOptions, PrimitiveKind } from './parser/threejs/index.js';

export { createViewport } from './viewport.js';
export type { Viewport, ViewportOpts } from './viewport.js';

export { buildBodyIdx, applyTransforms, flattenBodies, restoreBodies } from './sync.js';
export type { BodySnapshot } from './sync.js';
export { loadBodies, connectSim } from './sync.js';
export type { BodyInfo, LoadBodiesOpts, LoadBodiesResult, SimOpts, SimHandle } from './sync.js';
