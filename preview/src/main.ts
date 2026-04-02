import * as THREE from 'three';
import { createViewport, buildScene, loadBodies, connectSim, applyTransforms } from '@simarena/viewport';
import { expand } from '@simarena/format';
import { parseScene } from '@simarena/format';
import type { AssetDoc } from '@simarena/format';
import { buildTree } from './tree.js';

// ── DOM ───────────────────────────────────────────────────────────────────────

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const viewport = document.getElementById('viewport') as HTMLDivElement;
const tree = document.getElementById('tree') as HTMLDivElement;
const sceneName = document.getElementById('scene-name') as HTMLSpanElement;
const status = document.getElementById('status') as HTMLSpanElement;
const fileInput = document.getElementById('file-input') as HTMLInputElement;

// ── Viewport setup ────────────────────────────────────────────────────────────

const vp = createViewport(canvas, viewport);
vp.scene.background = new THREE.Color(0x111111);
vp.scene.fog = new THREE.Fog(0x111111, 10, 50);

const ambient = new THREE.AmbientLight(0xffffff, 0.6);
vp.scene.add(ambient);
const sun = new THREE.DirectionalLight(0xffffff, 0.8);
sun.position.set(5, 10, 5);
sun.castShadow = true;
vp.scene.add(sun);

(function loop() {
	requestAnimationFrame(loop);
	vp.orbit.update();
	vp.renderer.render(vp.scene, vp.camera);
})();

// ── Scene tracking ────────────────────────────────────────────────────────────

const permanent = new Set<THREE.Object3D>(vp.scene.children);

function clearScene() {
	for (const obj of [...vp.scene.children]) {
		if (!permanent.has(obj)) vp.scene.remove(obj);
	}
}

// ── SceneDoc loader ───────────────────────────────────────────────────────────

async function load(raw: unknown, base: string, label: string) {
	status.textContent = 'Parsing…';
	const parsed = parseScene(raw);
	if (!parsed.success) {
		status.textContent = `Invalid: ${parsed.errors?.[0] ?? 'unknown'}`;
		return;
	}

	status.textContent = 'Expanding…';
	const fetcher = (url: string): Promise<AssetDoc> => fetch(url).then(r => r.json());

	let doc;
	try {
		doc = await expand(parsed.data!, base, fetcher);
	} catch (e) {
		status.textContent = `Expand failed: ${e}`;
		return;
	}

	status.textContent = 'Building…';
	clearScene();

	try {
		const { bodies } = await buildScene(doc, vp.scene, base);
		buildTree(doc, bodies, tree, vp);
		sceneName.textContent = doc.name;
		document.title = `${doc.name} — preview`;
		status.textContent = `${doc.scene?.length ?? 0} entities`;
	} catch (e) {
		status.textContent = `Build failed: ${e}`;
		console.error(e);
	}
}

// ── URL param: ?url=... ───────────────────────────────────────────────────────

const urlParam = new URLSearchParams(location.search).get('url');
if (urlParam) {
	const base = urlParam.substring(0, urlParam.lastIndexOf('/') + 1);
	fetch(urlParam)
		.then(r => r.json())
		.then(raw => load(raw, base, urlParam))
		.catch(e => { status.textContent = `Fetch failed: ${e}`; });
}

// ── File input ────────────────────────────────────────────────────────────────

fileInput.addEventListener('change', () => {
	const file = fileInput.files?.[0];
	if (!file) return;
	const reader = new FileReader();
	reader.onload = () => {
		const raw = JSON.parse(reader.result as string);
		load(raw, '', file.name);
	};
	reader.readAsText(file);
});

// ── Drag & drop ───────────────────────────────────────────────────────────────

const dragOverlay = document.createElement('div');
dragOverlay.id = 'drag-overlay';
dragOverlay.textContent = 'Drop .scene.json here';
viewport.appendChild(dragOverlay);

document.addEventListener('dragover', e => { e.preventDefault(); dragOverlay.classList.add('active'); });
document.addEventListener('dragleave', e => { if (!e.relatedTarget) dragOverlay.classList.remove('active'); });
document.addEventListener('drop', e => {
	e.preventDefault();
	dragOverlay.classList.remove('active');
	const file = e.dataTransfer?.files[0];
	if (!file?.name.endsWith('.json')) return;
	const reader = new FileReader();
	reader.onload = () => {
		const raw = JSON.parse(reader.result as string);
		load(raw, '', file.name);
	};
	reader.readAsText(file);
});

// ── WebSocket mode: ?ws=ws://host:port/ws ─────────────────────────────────────
// Connects to a sim server (Python viewport or cloud) streaming body GLBs + transforms.
// Uses loadBodies + connectSim + applyTransforms from @simarena/viewport.

const wsParam = new URLSearchParams(location.search).get('ws');
if (wsParam) {
	const httpBase = wsParam.replace(/^ws(s?):\/\//, 'http$1://').replace(/\/ws$/, '');

	status.textContent = 'Loading…';
	sceneName.textContent = 'Live Sim';
	document.title = 'Live Sim — preview';
	clearScene();

	loadBodies(httpBase, vp.scene, {
		onProgress: (loaded, total) => { status.textContent = `Loading ${loaded}/${total}…`; }
	}).then(({ bodies, manifest }) => {
		const bodyIdx = new Map(manifest.map(b => [b.name, b.id] as [string, number]));

		// Particle system (lazy-created on first frame with particles)
		let particles: THREE.Points | null = null;
		let particleGeo: THREE.BufferGeometry | null = null;
		const particleMat = new THREE.PointsMaterial({ color: 0x3399ff, size: 0.03, transparent: true, opacity: 0.8, sizeAttenuation: true });

		connectSim(wsParam, manifest.length, (xpos, xquat) => {
			applyTransforms(xpos, xquat, bodyIdx, bodies);
		}, {
			onConnect: () => { status.textContent = '● Connected'; },
			onFps: fps => { status.textContent = `● ${fps} fps`; },
			onError: msg => { status.textContent = `✗ ${msg}`; },
			onClose: () => { status.textContent = '✗ Disconnected'; },
			onParticles: (positions) => {
				if (particles && particleGeo) {
					particleGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
				} else {
					particleGeo = new THREE.BufferGeometry();
					particleGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
					particles = new THREE.Points(particleGeo, particleMat);
					vp.scene.add(particles);
				}
			},
		});
	}).catch(e => { status.textContent = `Failed: ${e}`; });
}
