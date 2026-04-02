import * as THREE from 'three';
import type { SceneDoc, DocEntity, Physics, Joint } from '@simarena/format';
import type { Vec3, Geom } from '@simarena/format';

export type PrimitiveKind = 'box' | 'sphere' | 'cylinder' | 'capsule' | 'plane';

export interface ThreeEntity {
	name: string;
	obj: THREE.Object3D;
	kind: 'primitive' | 'robot';
	geom?: PrimitiveKind;
	size?: Vec3;
	color?: string;
	asset?: string;
	physics?: { dynamic?: boolean; mass?: number; friction?: number };
	joint?: { type: 'free' | 'hinge' | 'slide' | 'fixed' };
}

export interface ParseOptions {
	name?: string;
	gravity?: Vec3;
	timestep?: number;
	substeps?: number;
}

function buildGeom(kind: PrimitiveKind | undefined, size: Vec3, color?: string): Geom {
	const [a, b, c] = size;
	switch (kind) {
		case 'sphere':   return { type: 'sphere', radius: a, color, group: 'default' };
		case 'cylinder': return { type: 'cylinder', radius: a, height: b, color, group: 'default' };
		case 'capsule':  return { type: 'capsule', radius: a, height: b, color, group: 'default' };
		case 'plane':    return { type: 'plane', size: [a, b], color, group: 'default' };
		default:         return { type: 'box', size: [a, b, c] as Vec3, color, group: 'default' };
	}
}

export function parseThreeScene(entities: ThreeEntity[], opts: ParseOptions = {}): SceneDoc {
	const scene: DocEntity[] = entities.map(e => {
		const pos = e.obj.position.toArray() as Vec3;
		const q = e.obj.quaternion;
		const hasRot = Math.abs(q.x) + Math.abs(q.y) + Math.abs(q.z) > 1e-6;
		const quat = hasRot ? [q.w, q.x, q.y, q.z] as [number,number,number,number] : undefined;
		const transform = { pos, ...(quat ? { quat } : {}) };

		if (e.kind === 'robot' && e.asset) {
			const scl = (e.obj.scale.x + e.obj.scale.y + e.obj.scale.z) / 3;
			return {
				name: e.name,
				asset: e.asset,
				transform: { ...transform, ...(Math.abs(scl - 1) > 0.001 ? { scale: scl } : {}) }
			} satisfies DocEntity;
		}

		const dyn = e.physics?.dynamic ?? false;
		const physics: Physics = {
			dynamic: dyn,
			...(e.physics?.mass !== undefined ? { mass: e.physics.mass } : {}),
			...(e.physics?.friction !== undefined ? { friction: e.physics.friction } : {})
		};
		const joint: Joint | undefined = e.joint
			? { type: e.joint.type }
			: dyn ? { type: 'free' } : undefined;

		return {
			name: e.name,
			transform,
			geoms: [buildGeom(e.geom, e.size ?? [0.1, 0.1, 0.1], e.color)],
			physics,
			collider: {},
			...(joint ? { joint } : {})
		} satisfies DocEntity;
	});

	return {
		_format: 'simarena/scene@1.0',
		name: opts.name ?? 'Untitled Scene',
		sim: {
			timestep: opts.timestep ?? 0.002,
			gravity: opts.gravity ?? [0, 0, -9.81],
			substeps: opts.substeps ?? 4
		},
		scene
	};
}
