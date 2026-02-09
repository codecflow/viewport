import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
scene.fog = new THREE.Fog(0x111111, 10, 50);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
camera.position.set(3, 3, 3);

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('canvas'), antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 10, 5);
dirLight.castShadow = true;
scene.add(dirLight);

// Grid
const gridHelper = new THREE.GridHelper(10, 20, 0x444444, 0x222222);
scene.add(gridHelper);

// MuJoCo root: rotate -90° around X to convert Z-up to Y-up
const mujocoRoot = new THREE.Group();
mujocoRoot.rotation.x = -Math.PI / 2;
scene.add(mujocoRoot);

// Bodies storage
const bodies = new Map();
const loader = new GLTFLoader();

// Particle system for water/MPM
let particleSystem = null;
let particleGeometry = null;
const particleMaterial = new THREE.PointsMaterial({
    color: 0x3399ff,
    size: 0.03,
    transparent: true,
    opacity: 0.8,
    sizeAttenuation: true,
});

// Fetch and load bodies
async function loadBodies() {
    const response = await fetch('/bodies');
    const manifest = await response.json();
    
    document.getElementById('status').textContent = `Loading ${manifest.length} bodies...`;

    for (const bodyInfo of manifest) {
        const url = `/body/${bodyInfo.id}.glb`;
        
        loader.load(url, (gltf) => {
            const mesh = gltf.scene;
            mesh.traverse((child) => {
                if (child.isMesh) {
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            });
            
            mujocoRoot.add(mesh);
            bodies.set(bodyInfo.id, mesh);
            
            console.log(`Loaded body ${bodyInfo.id}: ${bodyInfo.name}`);
            
            if (bodies.size === manifest.length) {
                document.getElementById('status').textContent = '✓ Connected';
                connectWebSocket();
            }
        });
    }
}

// WebSocket for streaming transforms
let ws;
let frameCount = 0;
let lastTime = performance.now();

function connectWebSocket() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';

    ws.onmessage = (event) => {
        const buffer = new Float32Array(event.data);
        const nbody = bodies.size;
        
        // Expected size for transforms only: nbody*3 + nbody*4 = nbody*7
        const transformSize = nbody * 7;
        
        // Update body transforms
        for (let i = 0; i < nbody; i++) {
            const mesh = bodies.get(i);
            if (!mesh) continue;

            // Position (MuJoCo coordinates directly)
            const px = buffer[i * 3];
            const py = buffer[i * 3 + 1];
            const pz = buffer[i * 3 + 2];
            mesh.position.set(px, py, pz);

            // Quaternion (MuJoCo wxyz → Three.js xyzw)
            const qOffset = nbody * 3 + i * 4;
            const qw = buffer[qOffset];
            const qx = buffer[qOffset + 1];
            const qy = buffer[qOffset + 2];
            const qz = buffer[qOffset + 3];
            mesh.quaternion.set(qx, qy, qz, qw);
        }
        
        // Check for particle data (everything after transforms)
        if (buffer.length > transformSize) {
            const particleCount = buffer.length - transformSize;
            if (particleCount % 3 === 0) {
                const numParticles = particleCount / 3;
                const particlePositions = new Float32Array(particleCount);
                
                for (let i = 0; i < particleCount; i++) {
                    particlePositions[i] = buffer[transformSize + i];
                }
                
                // Update or create particle system
                if (!particleSystem) {
                    // Create new geometry and system
                    particleGeometry = new THREE.BufferGeometry();
                    particleGeometry.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));
                    particleSystem = new THREE.Points(particleGeometry, particleMaterial);
                    mujocoRoot.add(particleSystem);
                    console.log(`Created particle system with ${numParticles} particles`);
                } else {
                    // Update existing geometry (recreate buffer attribute for dynamic count)
                    particleGeometry.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));
                }
            }
        }

        // FPS counter
        frameCount++;
        const now = performance.now();
        if (now - lastTime >= 1000) {
            document.getElementById('stats').textContent = `${frameCount} FPS`;
            frameCount = 0;
            lastTime = now;
        }
    };

    ws.onerror = () => {
        document.getElementById('status').textContent = '✗ Connection Error';
    };

    ws.onclose = () => {
        document.getElementById('status').textContent = '✗ Disconnected';
    };
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Handle resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Start
loadBodies();
animate();
