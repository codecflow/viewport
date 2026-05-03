import * as THREE from 'three';
import type { Contact } from '../types.js';

const LAYER = 3;
const SPHERE_R = 0.015;
const ARROW_LEN = 0.12;
const FORCE_LOW = 0.5;
const FORCE_HIGH = 50;

const arrowDir = new THREE.Vector3();
const arrowOrigin = new THREE.Vector3();
const _color = new THREE.Color();

function forceColor(force: number): THREE.Color {
	const t = Math.min(1, Math.max(0, (force - FORCE_LOW) / (FORCE_HIGH - FORCE_LOW)));
	const hue = 0.6 - t * 0.6;
	return _color.clone().setHSL(hue, 1, 0.55);
}

function makeSphere(scene: THREE.Object3D, geo: THREE.SphereGeometry): THREE.Mesh {
	const mat = new THREE.MeshBasicMaterial({
		color: 0xffffff,
		depthTest: false,
		transparent: true,
		opacity: 0.85
	});
	const m = new THREE.Mesh(geo, mat);
	m.layers.set(LAYER);
	m.renderOrder = 999;
	m.visible = false;
	scene.add(m);
	return m;
}

function makeArrow(scene: THREE.Object3D): THREE.ArrowHelper {
	const a = new THREE.ArrowHelper(
		new THREE.Vector3(0, 0, 1),
		new THREE.Vector3(0, 0, 0),
		ARROW_LEN,
		0xffffff,
		ARROW_LEN * 0.3,
		ARROW_LEN * 0.15
	);
	a.layers.set(LAYER);
	a.line.layers.set(LAYER);
	a.cone.layers.set(LAYER);
	a.renderOrder = 999;
	a.visible = false;
	scene.add(a);
	return a;
}

export class ContactViz {
	#scene: THREE.Object3D;
	#geo: THREE.SphereGeometry;
	#spheres: THREE.Mesh[] = [];
	#arrows: THREE.ArrowHelper[] = [];
	visible = true;

	constructor(scene: THREE.Object3D) {
		this.#scene = scene;
		this.#geo = new THREE.SphereGeometry(SPHERE_R, 8, 6);
	}

	update(contacts: Contact[]): void {
		if (!this.visible) {
			this.hide();
			return;
		}

		while (this.#spheres.length < contacts.length) {
			this.#spheres.push(makeSphere(this.#scene, this.#geo));
			this.#arrows.push(makeArrow(this.#scene));
		}

		for (let i = 0; i < contacts.length; i++) {
			const c = contacts[i];
			if (!c) continue;
			const sphere = this.#spheres[i]!;
			const arrow = this.#arrows[i]!;
			const col = forceColor(c.force);

			sphere.position.set(c.pos[0], c.pos[1], c.pos[2]);
			(sphere.material as THREE.MeshBasicMaterial).color.copy(col);
			sphere.visible = true;

			if (c.normal) {
				arrowDir.set(c.normal[0], c.normal[1], c.normal[2]).normalize();
				arrowOrigin.set(c.pos[0], c.pos[1], c.pos[2]);
				arrow.position.copy(arrowOrigin);
				arrow.setDirection(arrowDir);
				arrow.setColor(col);
				const len = Math.min(ARROW_LEN * 3, ARROW_LEN + c.force * 0.002);
				arrow.setLength(len, len * 0.3, len * 0.15);
				arrow.visible = true;
			} else {
				arrow.visible = false;
			}
		}

		for (let i = contacts.length; i < this.#spheres.length; i++) {
			this.#spheres[i]!.visible = false;
			this.#arrows[i]!.visible = false;
		}
	}

	hide(): void {
		for (const s of this.#spheres) s.visible = false;
		for (const a of this.#arrows) a.visible = false;
	}

	dispose(): void {
		for (const s of this.#spheres) {
			(s.material as THREE.Material).dispose();
			this.#scene.remove(s);
		}
		for (const a of this.#arrows) {
			this.#scene.remove(a);
		}
		this.#spheres = [];
		this.#arrows = [];
		this.#geo.dispose();
	}
}
