// ── Engine-agnostic simulation state protocol ─────────────────────────────────
// Types live in @simarena/viewport (the bridge layer), not in engine adapters.
// Tier 1: transforms (universal)
// Tier 2: sensors, qpos (most engines)
// Tier 3: contacts (MuJoCo/Isaac only)

export interface SimScene {
	bodies: { name: string; id: number }[];
	actuators: { name: string; range: [number, number] }[];
	/** No engine-specific address offsets — adapters map internally. */
	sensors: { name: string; dim: number }[];
	/** Geom→body mapping for contact name resolution. */
	geoms: { name: string; body: string }[];
	timestep: number;
	nbody: number;
	nq: number;
	nu: number;
}

export interface Contact {
	geom1: string;
	geom2: string;
	pos: [number, number, number];
	/** Scalar contact force magnitude. */
	force: number;
	/** Contact frame normal (unit vector, first row of contact.frame). */
	normal?: [number, number, number];
}

export interface SimFrame {
	time: number;
	/** Flat world-space positions [x0,y0,z0, x1,...] per body. */
	xpos: Float32Array;
	/** Flat world-space quaternions wxyz [w0,x0,y0,z0, w1,...] per body. */
	xquat: Float32Array;
	qpos?: Float64Array;
	ctrl?: Float64Array;
	/** Named sensor readings: Record<sensorName, values[]>. */
	sensors?: Record<string, number[]>;
	/** Per-contact detail — Tier 3 only (MuJoCo/Isaac). */
	contacts?: Contact[];
	/** Engine-agnostic eval result — frontend narrows to EvalResult. */
	output?: unknown;
}

// ── Commands (consumer → engine) ──────────────────────────────────────────────
export type SimCommand =
	| { type: 'load'; scene: unknown }
	| { type: 'stop' }
	| { type: 'reset' }
	| { type: 'ctrl'; values: Float64Array }
	| { type: 'command'; values: Float32Array }
	| { type: 'setqpos'; qpos: Float64Array }
	| { type: 'pause' }
	| { type: 'resume' }
	| { type: 'configure'; contacts?: boolean; sensors?: boolean; output?: boolean; snapshotHz?: number; speed?: number };

// ── Events (engine → consumer) ────────────────────────────────────────────────
export type SimEvent =
	| { type: 'scene'; data: SimScene }
	| { type: 'frame'; data: SimFrame }
	| { type: 'status'; text: string }
	| { type: 'error'; message: string };

// ── Connection ────────────────────────────────────────────────────────────────
// A bidirectional channel to any sim engine — local Worker or remote server.
// Obtained from a transport-specific factory (connectWorker, connectWs, etc.).
export interface SimConnection {
	send(cmd: SimCommand): void;
	on(type: 'scene', cb: (data: SimScene) => void): void;
	on(type: 'frame', cb: (data: SimFrame) => void): void;
	on(type: 'status', cb: (text: string) => void): void;
	on(type: 'error', cb: (message: string) => void): void;
	off(type: string, cb: (...args: never[]) => void): void;
	close(): void;
}
