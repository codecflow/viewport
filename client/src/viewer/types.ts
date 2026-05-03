/** PBR material — engine-agnostic, all values 0-1 */
export interface MaterialDesc {
	color?: [number, number, number, number]; // RGBA 0-1
	roughness?: number;
	metalness?: number;
	mapUrl?: string; // diffuse texture URL (absolute)
}

/**
 * Geometry primitive.
 * size convention: box=[w,h,d] | sphere=[r] | cylinder=[r,h] | capsule=[r,h] | plane=[w,d] | mesh=[]
 */
export interface GeomDesc {
	type: string; // 'box' | 'sphere' | 'capsule' | 'cylinder' | 'plane' | 'mesh'
	size: number[];
	pos?: [number, number, number];
	rot?: [number, number, number]; // Euler degrees XYZ
	quat?: [number, number, number, number]; // WXYZ
	material?: MaterialDesc;
	meshUrl?: string; // absolute URL for STL/OBJ
}

/** Body/entity with its visual representation. */
export interface BodyDesc {
	name: string;
	pos: [number, number, number];
	rot?: [number, number, number]; // Euler degrees XYZ
	quat?: [number, number, number, number]; // WXYZ
	geoms: GeomDesc[];
	parent?: string; // parent entity name for hierarchy
}

export type LightType = 'point' | 'directional' | 'spot' | 'ambient' | 'hemisphere';

/** Light source — engine-agnostic description. */
export interface LightDesc {
	type: LightType;
	name: string;
	pos?: [number, number, number];
	color?: [number, number, number]; // RGB 0-1
	groundColor?: [number, number, number]; // hemisphere bottom color
	intensity?: number;
	castShadow?: boolean;
	dir?: [number, number, number]; // directional light direction vector
	range?: number; // point/spot falloff distance
	angle?: number; // spot cone half-angle (degrees)
}

/** Camera entity for thumbnail rendering. */
export interface SimCamera {
	name: string;
	fov: number;
	position: [number, number, number];
	lookAt: [number, number, number];
	entityName?: string;
	lookAtName?: string;
}

/** Sensor that needs visualization (lidar, etc.). */
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

/**
 * VisualScene — engine-agnostic visual description of a scene.
 *
 * Produced by @simarena/format (SceneDoc → VisualScene) or any simulation
 * engine (MuJoCo genesis, USD, etc.). Consumed by any renderer
 * (@simarena/viewport Three.js, future wgpu-rs headless, etc.).
 */
export interface VisualScene {
	bodies: BodyDesc[];
	lights: LightDesc[];
	cameras: SimCamera[];
	sensors: SensorDesc[];
}
