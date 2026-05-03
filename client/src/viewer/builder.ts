/**
 * VisualScene → Three.js scene builder.
 * Handles asset table resolution (materials, textures, meshes),
 * rigid/soft body branching, particles, and environment.
 */

import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import type {
	VisualScene, RigidBodyDesc, SoftBodyDesc, GeomDesc,
	LightDesc, MaterialDesc
} from './types.js';

// ── Asset caches ──────────────────────────────────────────────────────────────

const stlLoader = new STLLoader();
const gltfLoader = new GLTFLoader();
const geoCache = new Map<string, THREE.BufferGeometry>();
const objGroupCache = new Map<string, THREE.Group>();
const gltfGroupCache = new Map<string, THREE.Group>();
const mtlCreatorCache = new Map<string, MTLLoader.MaterialCreator | null>();
const texCache = new Map<string, THREE.Texture>();
const texLoader = new THREE.TextureLoader();

async function loadSTL(url: string): Promise<THREE.BufferGeometry> {
	const hit = geoCache.get(url);
	if (hit) return hit.clone();
	const geo = await stlLoader.loadAsync(url);
	geo.computeVertexNormals();
	geoCache.set(url, geo);
	return geo.clone();
}

async function loadMTL(url: string): Promise<MTLLoader.MaterialCreator | null> {
	if (mtlCreatorCache.has(url)) return mtlCreatorCache.get(url)!;
	try {
		const dir = url.substring(0, url.lastIndexOf('/') + 1);
		const loader = new MTLLoader();
		loader.setResourcePath(dir);
		const creator = await loader.loadAsync(url);
		creator.preload();
		mtlCreatorCache.set(url, creator);
		return creator;
	} catch {
		mtlCreatorCache.set(url, null);
		return null;
	}
}

async function loadOBJGroup(url: string): Promise<THREE.Group> {
	const mtlUrl = url.replace(/\.(obj|OBJ)$/, '.mtl');
	const creator = await loadMTL(mtlUrl);
	const cacheKey = url + (creator ? ':mtl' : '');
	const hit = objGroupCache.get(cacheKey);
	if (hit) return hit.clone() as THREE.Group;
	const loader = new OBJLoader();
	if (creator) loader.setMaterials(creator);
	const grp = await loader.loadAsync(url);
	grp.traverse((c) => {
		if (c instanceof THREE.Mesh) {
			const old = (Array.isArray(c.material) ? c.material[0] : c.material) as THREE.MeshPhongMaterial;
			c.material = new THREE.MeshStandardMaterial({
				map: old?.map ?? null,
				color: old?.map ? new THREE.Color(1, 1, 1) : (old?.color ?? new THREE.Color(0xaaaaaa)),
				metalness: 0.0, roughness: 0.8
			});
			c.castShadow = true;
			c.receiveShadow = true;
		}
	});
	objGroupCache.set(cacheKey, grp);
	return grp.clone() as THREE.Group;
}

async function loadGLTFGroup(url: string): Promise<THREE.Group> {
	const hit = gltfGroupCache.get(url);
	if (hit) return hit.clone() as THREE.Group;
	const gltf = await gltfLoader.loadAsync(url);
	const grp = gltf.scene;
	grp.traverse((c) => {
		if (c instanceof THREE.Mesh) { c.castShadow = true; c.receiveShadow = true; }
	});
	gltfGroupCache.set(url, grp);
	return grp.clone() as THREE.Group;
}

async function loadTex(url: string): Promise<THREE.Texture> {
	const hit = texCache.get(url);
	if (hit) return hit;
	const tex = await texLoader.loadAsync(url);
	tex.colorSpace = THREE.SRGBColorSpace;
	texCache.set(url, tex);
	return tex;
}

// ── Asset resolution ──────────────────────────────────────────────────────────

/** Resolve geom material — string ref OR inline MaterialDesc. */
function resolveMat(
	mat: string | MaterialDesc | undefined,
	visual: VisualScene
): MaterialDesc | undefined {
	if (mat === undefined) return undefined;
	if (typeof mat === 'string') return visual.materials?.[mat];
	return mat;
}

/** Resolve geom mesh URL — name ref → meshes table → URL, or direct meshUrl. */
function resolveMeshUrl(geom: GeomDesc, visual: VisualScene): string | undefined {
	if (geom.mesh) return visual.meshes?.[geom.mesh];
	return geom.meshUrl;
}

/** Resolve texture URL from materials table entry. */
function resolveTexUrl(mat: MaterialDesc | undefined, visual: VisualScene): string | undefined {
	if (!mat?.texture) return undefined;
	return visual.textures?.[mat.texture] ?? mat.texture;
}

// ── Material + geometry builders ──────────────────────────────────────────────

function buildMat(desc: MaterialDesc | undefined): THREE.MeshStandardMaterial {
	const c = desc?.color;
	const a = c ? c[3] ?? 1 : 1;
	return new THREE.MeshStandardMaterial({
		color: c ? new THREE.Color(c[0], c[1], c[2]) : new THREE.Color(0x6b7280),
		opacity: a,
		transparent: a < 1,
		roughness: desc?.roughness ?? 0.6,
		metalness: desc?.metalness ?? 0.1
	});
}

async function applyTexture(mat: THREE.MeshStandardMaterial, texUrl?: string): Promise<void> {
	if (!texUrl) return;
	try {
		mat.map = await loadTex(texUrl);
		mat.color.set(0xffffff);
		mat.needsUpdate = true;
	} catch { /* skip */ }
}

function buildGeo(geom: GeomDesc): THREE.BufferGeometry | null {
	const s = geom.size;
	switch (geom.type) {
		case 'box': return new THREE.BoxGeometry(s[0] ?? 1, s[1] ?? 1, s[2] ?? 1);
		case 'sphere': return new THREE.SphereGeometry(s[0] ?? 0.5, 32, 32);
		case 'cylinder': {
			const geo = new THREE.CylinderGeometry(s[0] ?? 0.1, s[0] ?? 0.1, s[1] ?? 0.5, 32);
			geo.rotateX(Math.PI / 2);
			return geo;
		}
		case 'capsule': {
			const geo = new THREE.CapsuleGeometry(s[0] ?? 0.1, s[1] ?? 0.5, 16, 32);
			geo.rotateX(Math.PI / 2);
			return geo;
		}
		case 'plane': {
			const [w, d] = [s[0] ?? 10, s[1] ?? 10];
			return new THREE.PlaneGeometry(w * 2, d * 2);
		}
		default: return null;
	}
}

function applyTransform(obj: THREE.Object3D, geom: GeomDesc): void {
	if (geom.pos) obj.position.set(geom.pos[0], geom.pos[1], geom.pos[2]);
	if (geom.quat) {
		const [w, x, y, z] = geom.quat;
		obj.quaternion.set(x, y, z, w).normalize();
	} else if (geom.rot) {
		const D = Math.PI / 180;
		obj.rotation.set(geom.rot[0] * D, geom.rot[1] * D, geom.rot[2] * D);
	}
}

async function buildGeomObject(geom: GeomDesc, parent: THREE.Group, visual: VisualScene): Promise<void> {
	const meshUrl = resolveMeshUrl(geom, visual);
	if (geom.type === 'mesh' && meshUrl) {
		const mat = buildMat(resolveMat(geom.material, visual));
		await applyTexture(mat, resolveTexUrl(resolveMat(geom.material, visual), visual));
		try {
			if (/\.(glb|gltf)$/i.test(meshUrl)) {
				const grp = await loadGLTFGroup(meshUrl);
				applyTransform(grp, geom);
				parent.add(grp);
			} else if (/\.(obj|OBJ)$/.test(meshUrl)) {
				const grp = await loadOBJGroup(meshUrl);
				if (geom.material) {
					grp.traverse((c) => {
						if (c instanceof THREE.Mesh) { c.material = mat.clone(); c.castShadow = true; c.receiveShadow = true; }
					});
				} else {
					grp.traverse((c) => {
						if (c instanceof THREE.Mesh) { c.castShadow = true; c.receiveShadow = true; }
					});
				}
				applyTransform(grp, geom);
				parent.add(grp);
			} else {
				const geo = await loadSTL(meshUrl);
				const mesh = new THREE.Mesh(geo, mat);
				mesh.castShadow = true; mesh.receiveShadow = true;
				applyTransform(mesh, geom);
				parent.add(mesh);
			}
		} catch (e) { console.error(`[builder] mesh load failed: ${meshUrl}`, e); }
	} else {
		const geo = buildGeo(geom);
		if (!geo) return;
		const matDesc = resolveMat(geom.material, visual);
		const mat = buildMat(matDesc);
		await applyTexture(mat, resolveTexUrl(matDesc, visual));
		geo.computeVertexNormals();
		const mesh = new THREE.Mesh(geo, mat);
		mesh.castShadow = true; mesh.receiveShadow = true;
		applyTransform(mesh, geom);
		parent.add(mesh);
	}
}

function buildLight(desc: LightDesc): THREE.Light | null {
	const color = desc.color ? new THREE.Color(desc.color[0], desc.color[1], desc.color[2]) : new THREE.Color(0xffffff);
	const intensity = desc.intensity ?? 1;
	switch (desc.type) {
		case 'point': {
			const l = new THREE.PointLight(color, intensity, desc.range ?? 0);
			l.castShadow = desc.castShadow ?? false;
			return l;
		}
		case 'directional': {
			const l = new THREE.DirectionalLight(color, intensity);
			if (desc.dir) l.position.set(-desc.dir[0], -desc.dir[1], -desc.dir[2]).normalize().multiplyScalar(5);
			l.castShadow = desc.castShadow ?? false;
			return l;
		}
		case 'spot': {
			const angle = desc.angle ? desc.angle * Math.PI / 180 : Math.PI / 6;
			const l = new THREE.SpotLight(color, intensity, desc.range ?? 0, angle);
			l.castShadow = desc.castShadow ?? false;
			return l;
		}
		case 'ambient': return new THREE.AmbientLight(color, intensity);
		case 'hemisphere': {
			const ground = desc.groundColor
				? new THREE.Color(desc.groundColor[0], desc.groundColor[1], desc.groundColor[2])
				: new THREE.Color(0x545454);
			return new THREE.HemisphereLight(color, ground, intensity);
		}
		default: return null;
	}
}

// ── Public sync helpers ───────────────────────────────────────────────────────

/**
 * Synchronously build a Three.js mesh from a primitive GeomDesc.
 * Returns null for 'mesh' type (use buildGeomMesh for async loading).
 */
export function createPrimitiveMesh(geom: GeomDesc): THREE.Mesh | null {
	if (geom.type === 'mesh') return null;
	const geo = buildGeo(geom);
	if (!geo) return null;
	geo.computeVertexNormals();
	const mesh = new THREE.Mesh(geo, buildMat(typeof geom.material === 'object' ? geom.material : undefined));
	mesh.castShadow = true;
	mesh.receiveShadow = true;
	return mesh;
}

export function createLightFromDesc(desc: LightDesc): THREE.Light | null {
	return buildLight(desc);
}

// ── Public API ────────────────────────────────────────────────────────────────

export interface BuiltScene {
	/** All rigid body objects keyed by name. */
	bodies: Map<string, THREE.Object3D>;
	/** Root rigid bodies only (no parent). */
	entityRoots: Map<string, THREE.Object3D>;
	/** Particle point clouds keyed by name — update positions per frame. */
	particles: Map<string, THREE.Points>;
	/** Soft body meshes keyed by name — update vertex positions per frame. */
	softBodies: Map<string, THREE.Mesh>;
}

/**
 * Build a Three.js scene from a VisualScene.
 * Resolves asset table references, handles rigid/soft bodies, particles, and environment.
 */
export async function buildSceneFromVisual(visual: VisualScene, scene: THREE.Scene): Promise<BuiltScene> {
	const bodies = new Map<string, THREE.Object3D>();
	const roots = new Map<string, THREE.Object3D>();
	const particlesMap = new Map<string, THREE.Points>();
	const softBodiesMap = new Map<string, THREE.Mesh>();
	const deferred: Array<{ grp: THREE.Group; parentName: string }> = [];

	// ── Apply environment ──────────────────────────────────────────────────────
	if (visual.environment) {
		const env = visual.environment;
		if (env.background) {
			if (Array.isArray(env.background)) {
				scene.background = new THREE.Color(env.background[0], env.background[1], env.background[2]);
			} else {
				scene.background = new THREE.Color(env.background);
			}
		}
		if (env.fog) {
			const f = env.fog;
			scene.fog = new THREE.Fog(new THREE.Color(f.color[0], f.color[1], f.color[2]), f.near, f.far);
		}
	}

	// ── Build bodies ───────────────────────────────────────────────────────────
	for (const body of visual.bodies) {
		if (body.kind === 'rigid') {
			const grp = new THREE.Group();
			grp.name = body.name;
			grp.position.set(...body.pos);
			if (body.quat) {
				const [w, x, y, z] = body.quat;
				grp.quaternion.set(x, y, z, w).normalize();
			} else if (body.rot) {
				const D = Math.PI / 180;
				grp.rotation.set(body.rot[0] * D, body.rot[1] * D, body.rot[2] * D);
			}
			bodies.set(body.name, grp);
			if (body.parent) {
				deferred.push({ grp, parentName: body.parent });
			} else {
				scene.add(grp);
				roots.set(body.name, grp);
			}
			for (const geom of body.geoms) await buildGeomObject(geom, grp, visual);

		} else if (body.kind === 'soft') {
			const geo = new THREE.BufferGeometry();
			const positions = new Float32Array(body.vertexCount * 3);
			geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
			geo.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(body.vertexCount * 3), 3));
			if (body.indices) geo.setIndex(body.indices);
			const matDesc = resolveMat(body.material, visual);
			const mat = buildMat(matDesc);
			if (body.doubleSided) mat.side = THREE.DoubleSide;
			const mesh = new THREE.Mesh(geo, mat);
			mesh.name = body.name;
			mesh.frustumCulled = false;
			scene.add(mesh);
			softBodiesMap.set(body.name, mesh);
		}
	}

	// ── Resolve rigid body hierarchy ───────────────────────────────────────────
	const allNames = [...bodies.keys()];
	for (const { grp, parentName } of deferred) {
		const parent = bodies.get(parentName);
		if (parent) {
			parent.add(grp);
		} else {
			console.warn(`[builder] unresolved parent "${parentName}" for "${grp.name}". Available: [${allNames.slice(0, 10).join(', ')}${allNames.length > 10 ? '...' : ''}]`);
			scene.add(grp);
			roots.set(grp.name, grp);
		}
	}

	// ── Build lights ───────────────────────────────────────────────────────────
	for (const desc of visual.lights) {
		const light = buildLight(desc);
		if (!light) continue;
		if (desc.pos) light.position.set(...desc.pos);
		scene.add(light);
	}

	// ── Build particle systems ─────────────────────────────────────────────────
	for (const pd of visual.particles ?? []) {
		const geo = new THREE.BufferGeometry();
		geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
		const matDesc = resolveMat(pd.material, visual);
		const c = matDesc?.color ?? [0.2, 0.6, 0.95, 0.8];
		const ptsMat = new THREE.PointsMaterial({
			color: new THREE.Color(c[0], c[1], c[2]),
			size: pd.size ?? 0.03,
			transparent: (c[3] ?? 1) < 1,
			opacity: c[3] ?? 1,
			sizeAttenuation: pd.sizeAttenuation ?? true,
		});
		const pts = new THREE.Points(geo, ptsMat);
		pts.name = pd.name;
		pts.frustumCulled = false;
		scene.add(pts);
		particlesMap.set(pd.name, pts);
	}

	return { bodies, entityRoots: roots, particles: particlesMap, softBodies: softBodiesMap };
}

/**
 * Build a single Three.js mesh from a GeomDesc.
 * Useful for editor live-preview of individual geoms.
 */
export async function buildGeomMesh(geom: GeomDesc, visual: VisualScene = { bodies: [], lights: [], cameras: [], sensors: [] }): Promise<THREE.Mesh | null> {
	const meshUrl = resolveMeshUrl(geom, visual);
	if (geom.type === 'mesh') {
		if (!meshUrl) return null;
		const matDesc = resolveMat(geom.material, visual);
		const mat = buildMat(matDesc);
		await applyTexture(mat, resolveTexUrl(matDesc, visual));
		try {
			const geo = await loadSTL(meshUrl);
			const mesh = new THREE.Mesh(geo, mat);
			mesh.castShadow = true; mesh.receiveShadow = true;
			return mesh;
		} catch { return null; }
	}
	const geo = buildGeo(geom);
	if (!geo) return null;
	geo.computeVertexNormals();
	const matDesc = resolveMat(geom.material, visual);
	const mat = buildMat(matDesc);
	await applyTexture(mat, resolveTexUrl(matDesc, visual));
	const mesh = new THREE.Mesh(geo, mat);
	mesh.castShadow = true; mesh.receiveShadow = true;
	return mesh;
}
