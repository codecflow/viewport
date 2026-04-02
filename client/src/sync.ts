import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// ── Body index ─────────────────────────────────────────────────────────────────
// Maps entity names → frame indices (matches the order bodies appear in sim state).

export function buildBodyIdx(names: string[]): Map<string, number> {
	const m = new Map<string, number>();
	names.forEach((n, i) => m.set(n, i));
	return m;
}

// ── Apply world-space transforms ───────────────────────────────────────────────
// Works for any sim source: WASM worker, WebSocket, cloud.
// xpos: flat Float32Array of positions [x0,y0,z0, x1,y1,z1, ...]
// xquat: flat Float32Array of quaternions in wxyz order [w0,x0,y0,z0, ...]
// Bodies not in the map are skipped automatically.

export function applyTransforms(
	xpos: Float32Array,
	xquat: Float32Array,
	bodyIdx: Map<string, number>,
	bodies: Map<string, THREE.Object3D>
): void {
	for (const [name, idx] of bodyIdx) {
		const obj = bodies.get(name);
		if (!obj) continue;
		const p = idx * 3, q = idx * 4;
		obj.position.set(xpos[p]!, xpos[p + 1]!, xpos[p + 2]!);
		// MuJoCo wxyz → Three.js xyzw
		obj.quaternion.set(xquat[q + 1]!, xquat[q + 2]!, xquat[q + 3]!, xquat[q]!);
	}
}

// ── Hierarchy flattening ───────────────────────────────────────────────────────
// World-space transforms (from any sim) require bodies to be direct children
// of the scene root. Flatten before sync, restore after.

export interface BodySnapshot {
	parent: THREE.Object3D;
	pos: THREE.Vector3;
	quat: THREE.Quaternion;
	scale: THREE.Vector3;
}

export function flattenBodies(
	bodies: Map<string, THREE.Object3D>,
	scene: THREE.Scene
): Map<string, BodySnapshot> {
	const saved = new Map<string, BodySnapshot>();
	for (const [name, obj] of bodies) {
		saved.set(name, {
			parent: obj.parent ?? scene,
			pos: obj.position.clone(),
			quat: obj.quaternion.clone(),
			scale: obj.scale.clone(),
		});
		if (obj.parent !== scene) scene.attach(obj);
	}
	return saved;
}

export function restoreBodies(
	flatBodies: Map<string, THREE.Object3D>,
	saved: Map<string, BodySnapshot>
): void {
	for (const [name, obj] of flatBodies) {
		const snap = saved.get(name);
		if (snap) {
			snap.parent.add(obj);
			obj.position.copy(snap.pos);
			obj.quaternion.copy(snap.quat);
			obj.scale.copy(snap.scale);
		}
		obj.matrixWorldAutoUpdate = true;
		obj.matrixWorldNeedsUpdate = true;
	}
}

// ── HTTP body loading ──────────────────────────────────────────────────────────
// Fetches GLBs from the Python viewport server (or any compatible server).
// Returns Map<name, Object3D> keyed by entity name, matching SceneDoc entity names.

export interface BodyInfo {
	id: number;
	name: string;
	fixed?: boolean;
	size?: number;
}

export interface LoadBodiesOpts {
	onProgress?: (loaded: number, total: number) => void;
}

const loader = new GLTFLoader();

export interface LoadBodiesResult {
	bodies: Map<string, THREE.Object3D>;
	manifest: BodyInfo[];
}

export async function loadBodies(
	httpBase: string,
	parent: THREE.Object3D,
	opts: LoadBodiesOpts = {}
): Promise<LoadBodiesResult> {
	const base = httpBase.replace(/\/$/, '');
	const res = await fetch(`${base}/bodies`);
	const manifest: BodyInfo[] = await res.json();
	const bodies = new Map<string, THREE.Object3D>();

	await Promise.all(manifest.map(b =>
		new Promise<void>(resolve => {
			loader.load(`${base}/body/${b.id}.glb`, gltf => {
				const mesh = gltf.scene;
				mesh.traverse(c => {
					if ((c as THREE.Mesh).isMesh) {
						(c as THREE.Mesh).castShadow = true;
						(c as THREE.Mesh).receiveShadow = true;
					}
				});
				parent.add(mesh);
				bodies.set(b.name, mesh);
				opts.onProgress?.(bodies.size, manifest.length);
				resolve();
			});
		})
	));

	return { bodies, manifest };
}

// ── WebSocket sim transport ────────────────────────────────────────────────────
// Connects to any sim server streaming binary frames.
// Wire protocol: [xpos: nbody*3 floats | xquat: nbody*4 floats]
// Buffer length must be divisible by 7 (nbody*7 = pos+quat per body).

export interface SimOpts {
	onFps?: (fps: number) => void;
	onParticles?: (positions: Float32Array) => void;
	onConnect?: () => void;
	onError?: (msg: string) => void;
	onClose?: () => void;
}

export interface SimHandle {
	close(): void;
}

export function connectSim(
	wsUrl: string,
	nbody: number,
	onFrame: (xpos: Float32Array, xquat: Float32Array) => void,
	opts: SimOpts = {}
): SimHandle {
	const ws = new WebSocket(wsUrl);
	ws.binaryType = 'arraybuffer';

	let frames = 0;
	let lastTick = performance.now();

	ws.onopen = () => opts.onConnect?.();

	ws.onmessage = e => {
		const buf = new Float32Array(e.data as ArrayBuffer);
		const xpos = buf.subarray(0, nbody * 3);
		const xquat = buf.subarray(nbody * 3, nbody * 7);
		onFrame(xpos, xquat);

		// Optional particle data appended after transforms
		if (buf.length > nbody * 7 && opts.onParticles) {
			const rest = buf.subarray(nbody * 7);
			if (rest.length % 3 === 0) opts.onParticles(rest);
		}

		frames++;
		const now = performance.now();
		if (now - lastTick >= 1000) {
			opts.onFps?.(frames);
			frames = 0;
			lastTick = now;
		}
	};

	ws.onerror = () => opts.onError?.('WebSocket error');
	ws.onclose = () => opts.onClose?.();

	return { close: () => ws.close() };
}
