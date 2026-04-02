import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export interface ViewportOpts {
	fov?: number;
	near?: number;
	far?: number;
	pos?: [number, number, number];
	target?: [number, number, number];
	grid?: number;
}

export interface Viewport {
	scene: THREE.Scene;
	camera: THREE.PerspectiveCamera;
	renderer: THREE.WebGLRenderer;
	orbit: OrbitControls;
	grid: THREE.GridHelper | null;
	resize(): void;
	dispose(): void;
}

export function createViewport(
	canvas: HTMLCanvasElement,
	container: HTMLElement,
	opts: ViewportOpts = {}
): Viewport {
	const { fov = 60, near = 0.1, far = 100, pos = [1.5, -1.5, 1.5], target = [0, 0, 0.3], grid = 2 } = opts;

	THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

	const scene = new THREE.Scene();

	const camera = new THREE.PerspectiveCamera(fov, container.clientWidth / container.clientHeight, near, far);
	camera.position.set(...pos);
	camera.lookAt(...target);

	const gl = canvas.getContext('webgl2');
	const renderer = new THREE.WebGLRenderer({ canvas, context: gl || undefined, antialias: true });
	renderer.setSize(container.clientWidth, container.clientHeight);
	renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
	renderer.shadowMap.enabled = true;
	renderer.shadowMap.type = THREE.PCFSoftShadowMap;
	renderer.toneMapping = THREE.ACESFilmicToneMapping;
	renderer.toneMappingExposure = 1.0;
	renderer.outputColorSpace = THREE.SRGBColorSpace;

	const orbit = new OrbitControls(camera, renderer.domElement);
	orbit.enableDamping = true;
	orbit.dampingFactor = 0.05;
	orbit.target.set(...target);
	orbit.update();

	let gridHelper: THREE.GridHelper | null = null;
	if (grid > 0) {
		gridHelper = new THREE.GridHelper(grid * 10, 20, 0x444444, 0x222222);
		gridHelper.rotation.x = Math.PI / 2;
		scene.add(gridHelper);
	}

	function resize() {
		const w = container.clientWidth, h = container.clientHeight;
		camera.aspect = w / h;
		camera.updateProjectionMatrix();
		renderer.setSize(w, h);
	}

	const obs = new ResizeObserver(resize);
	obs.observe(container);

	function dispose() {
		obs.disconnect();
		renderer.dispose();
		orbit.dispose();
	}

	return { scene, camera, renderer, orbit, grid: gridHelper, resize, dispose };
}
