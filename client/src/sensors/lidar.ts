/**
 * LidarViz — Three.js visualization of LiDAR scan data.
 * Reads pre-computed distances from sim.sensordata and draws lines or points.
 * No raycasting — pure display component.
 */
import * as THREE from 'three';
import type { LidarScanConfig, ScanPattern } from '@simarena/format';
import { buildDirs } from '@simarena/format';

export type { LidarScanConfig, ScanPattern };

const LINE_COLOR = 0x00ff88;
const COLOR_LOW  = new THREE.Color().setHSL(0, 1, 0.6);
const COLOR_HIGH = new THREE.Color().setHSL(0.55, 1, 0.6);

export class LidarViz {
	readonly group: THREE.Group;
	readonly name: string;

	private dirs: Float64Array;
	private n: number;
	private range: number;
	private height: number;
	private pattern: ScanPattern;
	private channels: number;
	private azimuth: number;
	private body: THREE.Object3D;
	private lines: THREE.LineSegments | null = null;
	private points: THREE.Points | null = null;
	private lineMat: THREE.LineBasicMaterial;
	private ptsMat: THREE.PointsMaterial;

	constructor(name: string, body: THREE.Object3D, cfg: LidarScanConfig) {
		this.name = name;
		this.body = body;
		this.pattern = cfg.pattern ?? '2d';
		this.range = cfg.range?.[1] ?? 10;
		this.height = cfg.height ?? 0.3;
		this.channels = cfg.channels ?? 16;
		this.azimuth = cfg.horizontalSamples ?? 120;
		this.dirs = buildDirs(cfg);
		this.n = this.dirs.length / 3;

		this.lineMat = new THREE.LineBasicMaterial({ color: LINE_COLOR });
		this.ptsMat = new THREE.PointsMaterial({ size: 0.05, vertexColors: true, sizeAttenuation: true });

		this.group = new THREE.Group();
		this.group.frustumCulled = false;
	}

	update(dists: number[] | null): void {
		if (this.lines) { this.group.remove(this.lines); this.lines.geometry.dispose(); this.lines = null; }
		if (this.points) { this.group.remove(this.points); this.points.geometry.dispose(); this.points = null; }
		if (!dists || dists.length === 0) return;

		const origin = new THREE.Vector3();
		this.body.getWorldPosition(origin);
		origin.z += this.height;

		const pts: number[] = [], cols: number[] = [], linePts: number[] = [];

		for (let i = 0; i < Math.min(this.n, dists.length); i++) {
			const d = dists[i] ?? this.range;
			const dx = this.dirs[i * 3], dy = this.dirs[i * 3 + 1], dz = this.dirs[i * 3 + 2];
			const hx = origin.x + dx * d, hy = origin.y + dy * d, hz = origin.z + dz * d;

			if (this.pattern === '2d') {
				linePts.push(origin.x, origin.y, origin.z, hx, hy, hz);
			} else if (d < this.range) {
				pts.push(hx, hy, hz);
				const t = this.pattern === '3d'
					? (Math.floor(i / this.azimuth) / Math.max(1, this.channels - 1))
					: (d / this.range);
				const c = new THREE.Color().lerpColors(COLOR_LOW, COLOR_HIGH, t);
				cols.push(c.r, c.g, c.b);
			}
		}

		if (linePts.length > 0) {
			const geo = new THREE.BufferGeometry();
			geo.setAttribute('position', new THREE.Float32BufferAttribute(linePts, 3));
			this.lines = new THREE.LineSegments(geo, this.lineMat);
			this.lines.frustumCulled = false;
			this.group.add(this.lines);
		}
		if (pts.length > 0) {
			const geo = new THREE.BufferGeometry();
			geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
			geo.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
			this.points = new THREE.Points(geo, this.ptsMat);
			this.group.add(this.points);
		}
	}

	dispose(): void {
		this.lines?.geometry.dispose();
		this.points?.geometry.dispose();
		this.lineMat.dispose();
		this.ptsMat.dispose();
	}
}
