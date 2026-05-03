/**
 * VisualScene — engine-agnostic visual description of a scene.
 *
 * Designed to be consumed by any renderer (Three.js, wgpu-rs headless, etc.).
 * Produced by @simarena/format (SceneDoc → VisualScene) or any sim engine
 * (MuJoCo, Genesis, Newton, USD, ...).
 *
 * Asset tables (materials, textures, meshes) deduplicate shared resources.
 * GeomDesc references them by name; renderers build GPU resources once per
 * unique material/mesh and batch draw calls accordingly.
 */

// ── Asset tables ──────────────────────────────────────────────────────────────

/** PBR material — engine-agnostic, all values 0-1. */
export interface MaterialDesc {
    color?: [number, number, number, number]; // RGBA 0-1
    roughness?: number;
    metalness?: number;
    texture?: string;   // ref to VisualScene.textures
}

// ── Geometry ──────────────────────────────────────────────────────────────────

/**
 * Geometry primitive.
 * size convention: box=[w,h,d] | sphere=[r] | cylinder=[r,h] | capsule=[r,h] | plane=[w,d] | mesh=[]
 */
export interface GeomDesc {
    type: string; // 'box' | 'sphere' | 'capsule' | 'cylinder' | 'plane' | 'mesh'
    size: number[];
    pos?: [number, number, number];
    rot?: [number, number, number];     // Euler degrees XYZ
    quat?: [number, number, number, number]; // WXYZ
    material?: string | MaterialDesc;   // name ref OR inline MaterialDesc
    mesh?: string;                      // name ref to VisualScene.meshes
    meshUrl?: string;                   // direct URL (convenience; prefer mesh ref)
}

// ── Bodies ────────────────────────────────────────────────────────────────────

/** Rigid body — updated per frame via transform (pos + quat). */
export interface RigidBodyDesc {
    kind: 'rigid';
    name: string;
    pos: [number, number, number];
    rot?: [number, number, number];
    quat?: [number, number, number, number];
    geoms: GeomDesc[];
    parent?: string;
}

/** Deformable body — updated per frame via per-vertex positions. */
export interface SoftBodyDesc {
    kind: 'soft';
    name: string;
    /** Total vertex count — pre-allocate GPU buffers. */
    vertexCount: number;
    /** Initial topology mesh (GLB/OBJ). Triangle indices stay fixed. */
    mesh?: string;              // ref to VisualScene.meshes
    meshUrl?: string;           // direct URL fallback
    /** Inline triangle indices (alternative to mesh ref). */
    indices?: number[];
    material?: string | MaterialDesc;
    doubleSided?: boolean;
}

export type BodyDesc = RigidBodyDesc | SoftBodyDesc;

// ── Lights ────────────────────────────────────────────────────────────────────

export type LightType = 'point' | 'directional' | 'spot' | 'ambient' | 'hemisphere';

export interface LightDesc {
    type: LightType;
    name: string;
    pos?: [number, number, number];
    color?: [number, number, number];       // RGB 0-1
    groundColor?: [number, number, number]; // hemisphere bottom color
    intensity?: number;
    castShadow?: boolean;
    dir?: [number, number, number];
    range?: number;
    angle?: number;
}

// ── Cameras ───────────────────────────────────────────────────────────────────

export interface SimCamera {
    name: string;
    fov: number;
    position: [number, number, number];
    lookAt: [number, number, number];
    entityName?: string;
    lookAtName?: string;
}

// ── Sensors ───────────────────────────────────────────────────────────────────

export interface SensorDesc {
    name: string;
    type: string; // 'lidar' | 'camera' | ...
    parentBody?: string;
    channels?: number;
    horizontalSamples?: number;
    horizontalFov?: [number, number];
    verticalFov?: [number, number];
    range?: [number, number];
}

// ── Dynamic elements ──────────────────────────────────────────────────────────

/** Point cloud — MPM particles, granular, fluid, debug markers. */
export interface ParticleDesc {
    name: string;
    material?: string | MaterialDesc;
    size?: number;
    sizeAttenuation?: boolean;
}

// ── Environment ───────────────────────────────────────────────────────────────

export interface EnvironmentDesc {
    /** Background color [R,G,B] 0-1 or HDR/EXR URL. */
    background?: [number, number, number] | string;
    fog?: { color: [number, number, number]; near: number; far: number };
}

// ── Root ──────────────────────────────────────────────────────────────────────

export interface VisualScene {
    // ── Shared asset tables ──────────────────────────────────────────────────
    /** name → PBR material. Geoms reference by name for GPU batching. */
    materials?: Record<string, MaterialDesc>;
    /** name → texture URL. Materials reference by name. */
    textures?: Record<string, string>;
    /** name → mesh URL (.stl/.obj/.glb). Geoms reference by name. */
    meshes?: Record<string, string>;

    // ── Scene graph ──────────────────────────────────────────────────────────
    /** Rigid + soft bodies. Discriminate with body.kind. */
    bodies: BodyDesc[];
    lights: LightDesc[];
    cameras: SimCamera[];
    sensors: SensorDesc[];
    /** Per-frame point cloud channels (particles, fluid, granular). */
    particles?: ParticleDesc[];
    environment?: EnvironmentDesc;
}
