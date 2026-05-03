/** LiDAR ray direction generators — pure math, no dependencies. */

/** Horizontal ring — 2D planar scan (SICK, RPLiDAR, Hokuyo) */
export function dirs2D(samples = 360, hfov: [number, number] = [-180, 180]): Float64Array {
	const out = new Float64Array(samples * 3);
	const start = hfov[0] * Math.PI / 180;
	const span  = (hfov[1] - hfov[0]) * Math.PI / 180;
	for (let i = 0; i < samples; i++) {
		const a = start + (i / samples) * span;
		out[i * 3] = Math.cos(a); out[i * 3 + 1] = Math.sin(a); out[i * 3 + 2] = 0;
	}
	return out;
}

/** Multi-channel vertical rings — 3D spinning (Velodyne, Ouster) */
export function dirs3D(
	channels = 16, azimuth = 120,
	hfov: [number, number] = [-180, 180],
	vfov: [number, number] = [-15, 15]
): Float64Array {
	const n = channels * azimuth, out = new Float64Array(n * 3);
	const hStart = hfov[0] * Math.PI / 180, hSpan = (hfov[1] - hfov[0]) * Math.PI / 180;
	for (let c = 0; c < channels; c++) {
		const phi = (vfov[0] + (c / (channels - 1)) * (vfov[1] - vfov[0])) * Math.PI / 180;
		for (let a = 0; a < azimuth; a++) {
			const th = hStart + (a / azimuth) * hSpan;
			const i = c * azimuth + a;
			out[i * 3] = Math.cos(phi) * Math.cos(th);
			out[i * 3 + 1] = Math.cos(phi) * Math.sin(th);
			out[i * 3 + 2] = Math.sin(phi);
		}
	}
	return out;
}

/** Time-varying petal pattern (Livox Mid-360 style) */
export function dirsLivox(t: number, rays = 600, maxEl = 20): Float64Array {
	const out = new Float64Array(rays * 3), el = maxEl * Math.PI / 180;
	for (let i = 0; i < rays; i++) {
		const alpha = (i / rays) * Math.PI * 6 + t * 0.2;
		const az = alpha, e = Math.sin(alpha * 2.3) * el;
		out[i * 3] = Math.cos(e) * Math.cos(az);
		out[i * 3 + 1] = Math.cos(e) * Math.sin(az);
		out[i * 3 + 2] = Math.sin(e);
	}
	return out;
}

/** Perspective grid — flash / depth camera */
export function dirsFlash(w = 64, h = 48, fovDeg = 60): Float64Array {
	const n = w * h, out = new Float64Array(n * 3), fov = fovDeg * Math.PI / 180;
	for (let y = 0; y < h; y++) {
		for (let x = 0; x < w; x++) {
			const nx = (x / (w - 1)) * 2 - 1, ny = (y / (h - 1)) * 2 - 1;
			const ax = Math.atan(nx * Math.tan(fov / 2) * (w / h));
			const ay = Math.atan(ny * Math.tan(fov / 2));
			const i = y * w + x;
			out[i * 3] = Math.cos(ay) * Math.cos(ax);
			out[i * 3 + 1] = Math.sin(ax);
			out[i * 3 + 2] = -Math.sin(ay);
		}
	}
	return out;
}

export type ScanPattern = '2d' | '3d' | 'livox' | 'flash';

export interface LidarScanConfig {
	pattern?: ScanPattern;
	horizontalSamples?: number;
	horizontalFov?: [number, number];
	verticalFov?: [number, number];
	channels?: number;
	range?: [number, number];
	sampleRate?: number;
	height?: number;
	near?: number;
}

/** Build directions from a LidarScanConfig */
export function buildDirs(cfg: LidarScanConfig, t = 0): Float64Array {
	const pattern = cfg.pattern ?? '2d';
	if (pattern === '3d') return dirs3D(cfg.channels ?? 16, cfg.horizontalSamples ?? 120, cfg.horizontalFov ?? [-180, 180], cfg.verticalFov ?? [-15, 15]);
	if (pattern === 'livox') return dirsLivox(t, cfg.horizontalSamples ?? 600);
	if (pattern === 'flash') return dirsFlash();
	return dirs2D(cfg.horizontalSamples ?? 120, cfg.horizontalFov ?? [-180, 180]);
}
