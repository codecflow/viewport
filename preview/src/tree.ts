import * as THREE from 'three';
import type { SceneDoc, DocEntity } from '@simarena/schema';
import type { Viewport } from '@simarena/viewport';

type Bodies = Map<string, THREE.Object3D>;

function iconFor(e: DocEntity): string {
	if (e.light) return '💡';
	if (e.sensor) return '📷';
	if (e.joint) return '🔗';
	if (e.asset) return '📦';
	if (e.geoms?.length) return '⬡';
	return '○';
}

function depth(entities: DocEntity[], name: string): number {
	const e = entities.find(x => x.name === name);
	if (!e?.relation) return 0;
	return 1 + depth(entities, e.relation.parent);
}

function focusBody(body: THREE.Object3D, vp: Viewport): void {
	const box = new THREE.Box3().setFromObject(body);
	if (box.isEmpty()) {
		vp.orbit.target.copy(body.position);
	} else {
		const center = box.getCenter(new THREE.Vector3());
		const size = box.getSize(new THREE.Vector3()).length();
		vp.orbit.target.copy(center);
		const dir = vp.camera.position.clone().sub(center).normalize();
		vp.camera.position.copy(center.clone().addScaledVector(dir, size * 1.5 + 0.5));
	}
	vp.orbit.update();
}

export function buildTree(
	doc: SceneDoc,
	bodies: Bodies,
	container: HTMLElement,
	vp: Viewport
): void {
	container.innerHTML = '';
	let active: HTMLElement | null = null;

	const entities = doc.scene ?? [];

	for (const e of entities) {
		const d = depth(entities, e.name);
		const row = document.createElement('div');
		row.className = 'tree-item';
		row.style.paddingLeft = `${8 + d * 14}px`;
		row.innerHTML = `<span class="icon">${iconFor(e)}</span><span class="name" title="${e.name}">${e.name}</span>`;

		row.addEventListener('click', () => {
			active?.classList.remove('active');
			row.classList.add('active');
			active = row;
			const body = bodies.get(e.name);
			if (body) focusBody(body, vp);
		});

		container.appendChild(row);
	}
}
