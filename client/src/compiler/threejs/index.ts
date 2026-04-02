import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import type { SceneDoc, DocEntity, MaterialDef, MeshDef, TextureDef, LightComponent, CameraComponent } from '@simarena/format';
import type { Geom } from '@simarena/format';
import type { Vec3, Vec4 } from '@simarena/format';

const stlLoader = new STLLoader();
const geoCache = new Map<string, THREE.BufferGeometry>();

async function loadSTL(url: string): Promise<THREE.BufferGeometry> {
	const hit = geoCache.get(url);
	if (hit) return hit.clone();
	const geo = await stlLoader.loadAsync(url);
	geo.computeVertexNormals();
	geoCache.set(url, geo);
	return geo.clone();
}

const objGroupCache = new Map<string, THREE.Group>();
const mtlCreatorCache = new Map<string, MTLLoader.MaterialCreator | null>();

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

const texLoader = new THREE.TextureLoader();
const texCache = new Map<string, THREE.Texture>();

async function loadTex(url: string): Promise<THREE.Texture> {
	const hit = texCache.get(url);
	if (hit) return hit;
	const tex = await texLoader.loadAsync(url);
	tex.colorSpace = THREE.SRGBColorSpace;
	texCache.set(url, tex);
	return tex;
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
				metalness: 0.0,
				roughness: 0.8
			});
			c.castShadow = true;
			c.receiveShadow = true;
		}
	});
	objGroupCache.set(cacheKey, grp);
	return grp.clone() as THREE.Group;
}

function stdMat(color?: string, opacity?: number): THREE.MeshStandardMaterial {
	const c = color ? new THREE.Color(color) : new THREE.Color(0x6b7280);
	const a = opacity ?? 1;
	return new THREE.MeshStandardMaterial({ color: c, transparent: a < 1, opacity: a, metalness: 0.1, roughness: 0.6 });
}

function renderGeoms(entity: DocEntity, all: Geom[]): Geom[] {
	if (!entity.visual) return all;
	if (!entity.visual.groups?.length) return all;
	return all.filter(g => entity.visual!.groups!.includes(g.group ?? ''));
}

function buildGeo(g: Geom): THREE.BufferGeometry | null {
	switch (g.type) {
		case 'box': {
			const [w, h, d] = g.size;
			return new THREE.BoxGeometry(w, h, d);
		}
		case 'sphere': return new THREE.SphereGeometry(g.radius, 32, 32);
		case 'cylinder': {
			const geo = new THREE.CylinderGeometry(g.radius, g.radius, g.height, 32);
			geo.rotateX(Math.PI / 2);
			return geo;
		}
		case 'capsule': {
			const geo = new THREE.CapsuleGeometry(g.radius, g.height, 16, 32);
			geo.rotateX(Math.PI / 2);
			return geo;
		}
		case 'plane': {
			const [w, d] = g.size ?? [10, 10];
			return new THREE.PlaneGeometry(w * 2, d * 2);
		}
		default: return null;
	}
}

function applyT(
	obj: THREE.Object3D,
	t?: { pos?: Vec3; rot?: Vec3; quat?: Vec4; scale?: number }
): void {
	if (!t) return;
	if (t.pos) obj.position.set(t.pos[0], t.pos[1], t.pos[2]);
	if (t.quat) {
		const [w, x, y, z] = t.quat;
		obj.quaternion.set(x, y, z, w).normalize();
	} else if (t.rot) {
		const D = Math.PI / 180;
		obj.rotation.set(t.rot[0] * D, t.rot[1] * D, t.rot[2] * D);
	}
	if (t.scale !== undefined) obj.scale.setScalar(t.scale);
}

function resolveMat(g: Geom, matMap: Map<string, MaterialDef>): THREE.MeshStandardMaterial {
	const opacity = g.opacity ?? 1;
	if (g.material && matMap.has(g.material)) {
		const def = matMap.get(g.material)!;
		if (def.rgba) {
			const alpha = g.opacity ?? def.rgba[3] ?? 1;
			return new THREE.MeshStandardMaterial({
				color: new THREE.Color(def.rgba[0], def.rgba[1], def.rgba[2]),
				transparent: alpha < 1, opacity: alpha,
				metalness: def.metalness ?? 0.1, roughness: def.roughness ?? 0.6
			});
		}
		if (def.color) return stdMat(def.color, opacity);
	}
	if (g.color) return stdMat(g.color, opacity);
	return new THREE.MeshStandardMaterial({ color: 0x888888, metalness: 0.1, roughness: 0.6 });
}

export function buildLightComponent(lc: LightComponent): THREE.Light | null {
	const color = lc.color ? new THREE.Color(lc.color) : new THREE.Color(0xffffff);
	const intensity = lc.intensity ?? 1;
	switch (lc.type) {
		case 'point': {
			const l = new THREE.PointLight(color, intensity, lc.range ?? 0);
			l.castShadow = lc.castShadow ?? false;
			return l;
		}
		case 'directional': {
			const l = new THREE.DirectionalLight(color, intensity);
			if (lc.dir) l.position.set(-lc.dir[0], -lc.dir[1], -lc.dir[2]).normalize().multiplyScalar(5);
			l.castShadow = lc.castShadow ?? false;
			return l;
		}
		case 'spot': {
			const l = new THREE.SpotLight(color, intensity, lc.range ?? 0, lc.angle ? lc.angle * Math.PI / 180 : Math.PI / 6);
			l.castShadow = lc.castShadow ?? false;
			return l;
		}
		case 'ambient': return new THREE.AmbientLight(color, intensity);
		case 'hemisphere': {
			const g = lc.groundColor ? new THREE.Color(lc.groundColor) : new THREE.Color(0x545454);
			return new THREE.HemisphereLight(color, g, intensity);
		}
		default: return null;
	}
}

function buildMeshUrlMap(meshes: MeshDef[], basePath: string): Map<string, string> {
	const map = new Map<string, string>();
	for (const m of meshes) {
		const url = m.file.startsWith('/') || m.file.startsWith('http') ? m.file : basePath + m.file;
		map.set(m.name, url);
	}
	return map;
}

function buildMatMap(materials: MaterialDef[]): Map<string, MaterialDef> {
	const map = new Map<string, MaterialDef>();
	for (const m of materials) map.set(m.name, m);
	return map;
}

function buildTextureUrlMap(textures: TextureDef[], basePath: string): Map<string, string> {
	const map = new Map<string, string>();
	for (const t of textures) {
		const url = t.file.startsWith('/') || t.file.startsWith('http') ? t.file : basePath + t.file;
		map.set(t.name, url);
	}
	return map;
}

interface BuildCtx {
	bodies: Map<string, THREE.Object3D>;
	entityRoots: Map<string, THREE.Object3D>;
	meshUrls: Map<string, string>;
	textureUrls: Map<string, string>;
	materials: Map<string, MaterialDef>;
	threeScene: THREE.Scene;
	deferredParents: Array<{ group: THREE.Group; parentPath: string }>;
}

async function applyTexture(mat: THREE.MeshStandardMaterial, g: Geom, ctx: BuildCtx): Promise<void> {
	if (!g.material) return;
	const def = ctx.materials.get(g.material);
	if (!def?.map) return;
	const texUrl = ctx.textureUrls.get(def.map);
	if (!texUrl) return;
	try {
		mat.map = await loadTex(texUrl);
		mat.color.set(0xffffff);
		mat.needsUpdate = true;
	} catch { /* skip */ }
}

async function buildGeom(g: Geom, parent: THREE.Group, ctx: BuildCtx): Promise<void> {
	if (g.type === 'mesh') {
		const url = ctx.meshUrls.get(g.mesh);
		if (!url) { console.warn(`[threejs] mesh not in catalog: "${g.mesh}"`); return; }
		try {
			if (/\.(obj|OBJ)$/.test(url)) {
				const grp = await loadOBJGroup(url);
				if (g.material || g.color) {
					const mat = resolveMat(g, ctx.materials);
					await applyTexture(mat, g, ctx);
					grp.traverse((c) => { if (c instanceof THREE.Mesh) { c.material = mat.clone(); c.castShadow = true; c.receiveShadow = true; } });
				} else {
					grp.traverse((c) => { if (c instanceof THREE.Mesh) { c.castShadow = true; c.receiveShadow = true; } });
				}
				applyT(grp, { pos: g.pos, rot: g.rot, quat: g.quat });
				parent.add(grp);
			} else {
				const mat = resolveMat(g, ctx.materials);
				await applyTexture(mat, g, ctx);
				const geo = await loadSTL(url);
				geo.computeVertexNormals();
				const mesh = new THREE.Mesh(geo, mat);
				mesh.castShadow = true; mesh.receiveShadow = true;
				applyT(mesh, { pos: g.pos, rot: g.rot, quat: g.quat });
				parent.add(mesh);
			}
		} catch (e) { console.error(`[threejs] failed to load mesh "${g.mesh}" from ${url}`, e); }
	} else {
		const geo = buildGeo(g);
		if (!geo) return;
		const mat = resolveMat(g, ctx.materials);
		await applyTexture(mat, g, ctx);
		geo.computeVertexNormals();
		const mesh = new THREE.Mesh(geo, mat);
		mesh.castShadow = true; mesh.receiveShadow = true;
		applyT(mesh, { pos: g.pos, rot: g.rot, quat: g.quat });
		parent.add(mesh);
	}
}

function buildCameraIcon(cc: CameraComponent, parent: THREE.Group): void {
	const mat = new THREE.MeshStandardMaterial({ color: 0x00bcd4, metalness: 0.3, roughness: 0.5 });
	const body = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.065, 0.075), mat);
	body.castShadow = true;

	const lensMat = new THREE.MeshStandardMaterial({ color: 0x007a9e });
	const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.02, 0.05, 8), lensMat);
	lens.rotation.x = Math.PI / 2;
	lens.position.z = 0.06;

	const fov = (cc.fov ?? 50) * Math.PI / 180;
	const aspect = (cc.resolution?.[0] ?? 256) / (cc.resolution?.[1] ?? 256);
	const near = cc.near ?? 0.1;
	const far = Math.min(cc.far ?? 100, 2);
	const hNear = Math.tan(fov / 2) * near;
	const wNear = hNear * aspect;
	const hFar  = Math.tan(fov / 2) * far;
	const wFar  = hFar * aspect;

	const pts = [
		new THREE.Vector3(-wNear, -hNear, near), new THREE.Vector3( wNear, -hNear, near),
		new THREE.Vector3( wNear, -hNear, near), new THREE.Vector3( wNear,  hNear, near),
		new THREE.Vector3( wNear,  hNear, near), new THREE.Vector3(-wNear,  hNear, near),
		new THREE.Vector3(-wNear,  hNear, near), new THREE.Vector3(-wNear, -hNear, near),
		new THREE.Vector3(-wFar, -hFar, far), new THREE.Vector3( wFar, -hFar, far),
		new THREE.Vector3( wFar, -hFar, far), new THREE.Vector3( wFar,  hFar, far),
		new THREE.Vector3( wFar,  hFar, far), new THREE.Vector3(-wFar,  hFar, far),
		new THREE.Vector3(-wFar,  hFar, far), new THREE.Vector3(-wFar, -hFar, far),
		new THREE.Vector3(0, 0, 0), new THREE.Vector3(-wFar, -hFar, far),
		new THREE.Vector3(0, 0, 0), new THREE.Vector3( wFar, -hFar, far),
		new THREE.Vector3(0, 0, 0), new THREE.Vector3( wFar,  hFar, far),
		new THREE.Vector3(0, 0, 0), new THREE.Vector3(-wFar,  hFar, far),
	];
	const geo = new THREE.BufferGeometry().setFromPoints(pts);
	const lines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x00bcd4, opacity: 0.6, transparent: true }));
	lines.userData['isCameraIcon'] = true;

	parent.add(body);
	parent.add(lens);
	parent.add(lines);
}

async function buildEntity(entity: DocEntity, parentObject: THREE.Object3D, ctx: BuildCtx): Promise<void> {
	const grp = new THREE.Group();
	grp.name = entity.name;
	applyT(grp, entity.transform);
	ctx.bodies.set(entity.name, grp);

	const parentRef = entity.relation?.parent;
	if (parentRef) {
		ctx.deferredParents.push({ group: grp, parentPath: parentRef });
	} else {
		parentObject.add(grp);
	}

	for (const g of renderGeoms(entity, entity.geoms ?? [])) await buildGeom(g, grp, ctx);

	if (entity.light) {
		const l = buildLightComponent(entity.light);
		if (l) {
			if (entity.transform?.pos) l.position.set(...entity.transform.pos);
			grp.add(l);
		}
	}

	if (entity.camera) {
		buildCameraIcon(entity.camera, grp);
	}
}

export interface ViewerScene {
	bodies: Map<string, THREE.Object3D>;
	entityRoots: Map<string, THREE.Object3D>;
}

export async function buildScene(doc: SceneDoc, threeScene: THREE.Scene, basePath = ''): Promise<ViewerScene> {
	const bodies = new Map<string, THREE.Object3D>();
	const entityRoots = new Map<string, THREE.Object3D>();
	const deferredParents: Array<{ group: THREE.Group; parentPath: string }> = [];

	const ctx: BuildCtx = {
		bodies, entityRoots,
		meshUrls: buildMeshUrlMap(doc.meshes ?? [], basePath),
		textureUrls: buildTextureUrlMap(doc.textures ?? [], basePath),
		materials: buildMatMap(doc.materials ?? []),
		threeScene, deferredParents
	};

	for (const entity of doc.scene ?? []) {
		await buildEntity(entity, threeScene, ctx);
		if (!entity.relation) {
			const root = bodies.get(entity.name);
			if (root) entityRoots.set(entity.name, root);
		}
	}

	for (const { group, parentPath } of deferredParents) {
		const target = bodies.get(parentPath);
		if (target) {
			group.removeFromParent();
			target.add(group);
		} else {
			console.warn(`[threejs] parent ref unresolved: "${parentPath}" for "${group.name}"`);
			threeScene.add(group);
		}
	}

	return { bodies, entityRoots };
}

export function createGeomMesh(g: Geom, color?: string): THREE.Mesh | null {
	const geo = buildGeo(g);
	if (!geo) return null;
	geo.computeVertexNormals();
	const mat = new THREE.MeshStandardMaterial({
		color: color ?? g.color ?? '#6b7280',
		roughness: 0.6, metalness: 0.1
	});
	const mesh = new THREE.Mesh(geo, mat);
	mesh.castShadow = true;
	mesh.receiveShadow = true;
	return mesh;
}
