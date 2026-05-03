import * as THREE from 'three';
import { LidarViz } from '../sensors/index.js';
import { applyTransforms, buildBodyIdx } from '../sync.js';
import type { SimScene as SimSceneMsg } from '../types.js';
import { ContactViz } from '../sensors/contact.js';
import type { SimCamera, VisualScene } from './types.js';

/** The subset of state that SimScene needs for per-frame visualization. */
export interface VizState {
	showContacts: boolean;
	// eslint-disable-next-line @typescript-eslint/no-explicit-any
	contacts: any[];
	sensordata: Record<string, number[]> | null;
}

/**
 * SimScene — Three.js renderer for simulation visualization.
 *
 * Lifecycle:
 *   1. new SimScene(scene)
 *   2. setBodies(map)               — register body objects
 *   3. init(msg, visual)            — build bodyIdx, proxy meshes, lidars, thumb cameras
 *   4. Per frame: applyFrame() → updateViz() → renderThumbnails()
 *   5. dispose()
 */
export class SimScene {
	static readonly THUMB_W = 160;
	static readonly THUMB_H = 100;

	#scene: THREE.Scene;
	#bodies: Map<string, THREE.Object3D> = new Map();
	#bodyIdx: Map<string, number> = new Map();
	#proxies: THREE.Object3D[] = [];
	#lidars: LidarViz[] = [];
	#contactViz: ContactViz;
	#thumbCams: { name: string; cam: THREE.PerspectiveCamera; entityName?: string; lookAtName?: string }[] = [];
	#cameras: SimCamera[] = [];
	#thumbTarget: THREE.WebGLRenderTarget | null = null;

	/** Config list — updated on init(). */
	thumbConfigs: { key: string; label: string }[] = [];

	constructor(scene: THREE.Scene) {
		this.#scene = scene;
		this.#contactViz = new ContactViz(scene);
	}

	// ── Body management ────────────────────────────────────────────────────────

	setBodies(bodies: Map<string, THREE.Object3D>): void {
		this.#bodies = bodies;
	}

	get bodies(): Map<string, THREE.Object3D> {
		return this.#bodies;
	}

	// ── Lifecycle ──────────────────────────────────────────────────────────────

	init(msg: SimSceneMsg, visual: VisualScene): void {
		this.#bodyIdx.clear();
		buildBodyIdx(msg.bodies.map((b) => b.name)).forEach((v, k) => this.#bodyIdx.set(k, v));
		this.#cameras = visual.cameras;
		this.#initThumbCams(visual.cameras);
		this.#createProxies(visual);
		this.#createLidars(visual);
	}

	// ── Per-frame updates ──────────────────────────────────────────────────────

	applyFrame(xpos: Float32Array, xquat: Float32Array): void {
		applyTransforms(xpos, xquat, this.#bodyIdx, this.#bodies);
	}

	updateViz(state: VizState): void {
		this.#syncThumbCams();
		this.#contactViz.visible = state.showContacts;
		this.#contactViz.update(state.contacts as Parameters<ContactViz['update']>[0]);
		for (const viz of this.#lidars) viz.update(state.sensordata?.[viz.name] ?? null);
	}

	// ── Thumbnail cameras ──────────────────────────────────────────────────────

	renderThumbnails(
		renderer: THREE.WebGLRenderer,
		ctxs: (CanvasRenderingContext2D | null)[],
		running: boolean,
		time: number
	): void {
		if (!running || !this.#thumbCams.length) return;
		if (Math.round(time * 50) % 3 !== 0) return;
		const W = SimScene.THUMB_W;
		const H = SimScene.THUMB_H;
		const target = this.#getThumbTarget();
		const prev = renderer.outputColorSpace;
		renderer.outputColorSpace = THREE.SRGBColorSpace;
		for (let i = 0; i < this.#thumbCams.length; i++) {
			const ctx2d = ctxs[i];
			if (!ctx2d) continue;
			renderer.setRenderTarget(target);
			renderer.render(this.#scene, this.#thumbCams[i]!.cam);
			renderer.setRenderTarget(null);
			const buf = new Uint8Array(W * H * 4);
			renderer.readRenderTargetPixels(target, 0, 0, W, H, buf);
			const img = ctx2d.createImageData(W, H);
			for (let y = 0; y < H; y++) {
				const src = (H - 1 - y) * W * 4;
				img.data.set(buf.subarray(src, src + W * 4), y * W * 4);
			}
			ctx2d.putImageData(img, 0, 0);
		}
		renderer.outputColorSpace = prev;
	}

	getThumbCam(key: string): THREE.PerspectiveCamera | null {
		return this.#thumbCams.find((c) => c.name === key)?.cam ?? null;
	}

	getThumbConfig(key: string): SimCamera | null {
		return this.#cameras.find((c) => c.name === key) ?? null;
	}

	// ── Dispose ────────────────────────────────────────────────────────────────

	dispose(): void {
		for (const viz of this.#lidars) {
			this.#scene.remove(viz.group);
			viz.dispose();
		}
		this.#lidars.length = 0;

		for (const obj of this.#proxies) {
			this.#scene.remove(obj);
			obj.traverse((c) => {
				if (c instanceof THREE.Mesh) {
					c.geometry.dispose();
					(c.material as THREE.Material).dispose();
				}
			});
		}
		this.#proxies.length = 0;

		this.#contactViz.dispose();
		this.#thumbCams = [];
		this.#cameras = [];
		this.thumbConfigs = [];
		this.#thumbTarget?.dispose();
		this.#thumbTarget = null;
		this.#bodyIdx.clear();
	}

	// ── Private ────────────────────────────────────────────────────────────────

	#getThumbTarget(): THREE.WebGLRenderTarget {
		if (!this.#thumbTarget) {
			this.#thumbTarget = new THREE.WebGLRenderTarget(SimScene.THUMB_W, SimScene.THUMB_H, {
				minFilter: THREE.LinearFilter,
				magFilter: THREE.LinearFilter,
				format: THREE.RGBAFormat,
				type: THREE.UnsignedByteType
			});
			this.#thumbTarget.texture.colorSpace = THREE.SRGBColorSpace;
		}
		return this.#thumbTarget;
	}

	#syncThumbCams(): void {
		for (const tc of this.#thumbCams) {
			if (!tc.entityName) continue;
			const obj = this.#bodies.get(tc.entityName);
			if (obj) {
				tc.cam.position.copy(obj.position);
				tc.cam.quaternion.copy(obj.quaternion);
			}
			if (tc.lookAtName) {
				const target = this.#bodies.get(tc.lookAtName);
				if (target) tc.cam.lookAt(target.position);
			}
		}
	}

	#initThumbCams(cameras: SimCamera[]): void {
		const W = SimScene.THUMB_W;
		const H = SimScene.THUMB_H;
		this.#thumbCams = cameras.map((c) => {
			const cam = new THREE.PerspectiveCamera(c.fov, W / H, 0.1, 100);
			cam.position.set(...c.position);
			cam.lookAt(...c.lookAt);
			return { name: c.name, cam, entityName: c.entityName, lookAtName: c.lookAtName };
		});
		this.thumbConfigs = this.#thumbCams.map((c) => ({ key: c.name, label: c.name }));
	}

	#createProxies(visual: VisualScene): void {
		for (const body of visual.bodies) {
			if (this.#bodies.has(body.name)) continue;
			const grp = new THREE.Group();
			grp.position.set(...body.pos);
			if (body.quat) {
				const [w, x, y, z] = body.quat;
				grp.quaternion.set(x, y, z, w);
			}
			for (const geom of body.geoms) {
				if (geom.type === 'box') {
					const [sx, sy, sz] = geom.size as [number, number, number];
					const mesh = new THREE.Mesh(
						new THREE.BoxGeometry(sx, sy, sz),
						new THREE.MeshStandardMaterial({
							color: 0x1f1f3e,
							roughness: 0.7,
							metalness: 0.2,
							transparent: true,
							opacity: 0.9
						})
					);
					grp.add(mesh);
				}
				// TODO: add sphere, cylinder, capsule, mesh (GLB) handling
			}
			if (grp.children.length > 0) {
				this.#scene.add(grp);
				this.#bodies.set(body.name, grp);
				this.#proxies.push(grp);
			}
		}
	}

	#createLidars(visual: VisualScene): void {
		for (const s of visual.sensors) {
			if (s.type !== 'lidar') continue;
			const body = this.#bodies.get(s.parentBody ?? s.name);
			if (!body) continue;
			const ch = s.channels ?? 1;
			const viz = new LidarViz(s.name, body, {
				pattern: ch > 1 ? '3d' : '2d',
				channels: ch,
				horizontalSamples: s.horizontalSamples ?? 120,
				horizontalFov: s.horizontalFov ?? [-180, 180],
				verticalFov: s.verticalFov as [number, number] | undefined,
				range: s.range ?? [0.1, 10],
				height: 0.3
			});
			this.#scene.add(viz.group);
			this.#lidars.push(viz);
		}
	}
}
