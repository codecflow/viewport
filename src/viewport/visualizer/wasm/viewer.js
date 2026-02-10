import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
camera.position.set(2, 2, 2);

const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('canvas'), antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 10, 5);
dirLight.castShadow = true;
scene.add(dirLight);

// Grid
scene.add(new THREE.GridHelper(10, 20, 0x444444, 0x222222));

// MuJoCo root
const mujocoRoot = new THREE.Group();
scene.add(mujocoRoot);

// State
let mujoco = null;
let model = null;
let data = null;
const bodies = {};

let isPlaying = true;
let frameCount = 0;
let lastTime = performance.now();

// UI
const loadingDiv = document.getElementById('loading');
const uiDiv = document.getElementById('ui');
const statsSpan = document.getElementById('stats');
const statusSpan = document.getElementById('status');
const playPauseBtn = document.getElementById('playPause');
const resetBtn = document.getElementById('reset');
const stepBtn = document.getElementById('step');

// Load MuJoCo WASM
async function loadMuJoCo() {
    loadingDiv.querySelector('div:last-child').textContent = 'Loading MuJoCo WASM...';
    
    const { default: load_mujoco } = await import('./mujoco.js');
    mujoco = await load_mujoco();
    
    // Setup virtual filesystem
    mujoco.FS.mkdir('/working');
    mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
    
    console.log('MuJoCo WASM loaded');
    return mujoco;
}

// Download files from manifest
async function downloadFiles() {
    loadingDiv.querySelector('div:last-child').textContent = 'Loading manifest...';
    
    const manifestResponse = await fetch('manifest.json');
    const manifest = await manifestResponse.json();
    
    document.title = manifest.title;
    loadingDiv.querySelector('div:last-child').textContent = `Downloading ${manifest.files.length} files...`;
    
    // Download all files
    const requests = manifest.files.map(filename => fetch(filename));
    const responses = await Promise.all(requests);
    
    for (let i = 0; i < responses.length; i++) {
        if (!responses[i].ok) {
            throw new Error(`Failed to fetch ${manifest.files[i]}: ${responses[i].status}`);
        }
        
        const filename = manifest.files[i];
        const split = filename.split('/');
        let working = '/working/';
        
        // Create directory structure
        for (let f = 0; f < split.length - 1; f++) {
            working += split[f];
            if (!mujoco.FS.analyzePath(working).exists) {
                mujoco.FS.mkdir(working);
            }
            working += '/';
        }
        
        // Write file
        const filepath = '/working/' + filename;
        mujoco.FS.writeFile(filepath, new Uint8Array(await responses[i].arrayBuffer()));
        console.log(`Written: ${filepath}`);
    }
    
    return manifest;
}

// Build scene from MuJoCo model
function buildScene() {
    loadingDiv.querySelector('div:last-child').textContent = 'Building scene...';
    
    // Load model
    model = mujoco.MjModel.loadFromXML('/working/scene.xml');
    data = new mujoco.MjData(model);
    
    // Run forward kinematics
    mujoco.mj_forward(model, data);
    
    console.log(`Model loaded: ${model.nbody} bodies, ${model.ngeom} geoms`);
    
    // Decode names
    const textDecoder = new TextDecoder('utf-8');
    const names = new Uint8Array(model.names);
    
    // Build geometries
    for (let g = 0; g < model.ngeom; g++) {
        // Only visualize geom groups 0-2
        if (model.geom_group[g] >= 3) continue;
        
        const bodyId = model.geom_bodyid[g];
        const type = model.geom_type[g];
        
        const size = [
            model.geom_size[g * 3 + 0],
            model.geom_size[g * 3 + 1],
            model.geom_size[g * 3 + 2]
        ];
        
        // Create body group if needed
        if (!(bodyId in bodies)) {
            bodies[bodyId] = new THREE.Group();
            bodies[bodyId].userData.bodyID = bodyId;
            
            // Get body name
            const nameStart = model.name_bodyadr[bodyId];
            let nameEnd = nameStart;
            while (nameEnd < names.length && names[nameEnd] !== 0) nameEnd++;
            bodies[bodyId].name = textDecoder.decode(names.subarray(nameStart, nameEnd));
        }
        
        // Create geometry
        const geometry = createGeometry(mujoco, type, size);
        const material = createMaterial(model, g);
        
        let mesh;
        if (type === 0) {
            // Plane
            mesh = new THREE.Mesh(new THREE.PlaneGeometry(100, 100), material);
            mesh.rotateX(-Math.PI / 2);
        } else {
            mesh = new THREE.Mesh(geometry, material);
        }
        
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.userData.bodyID = bodyId;
        bodies[bodyId].add(mesh);
        
        // Set position and quaternion
        getPosition(model.geom_pos, g, mesh.position);
        if (type !== 0) {
            getQuaternion(model.geom_quat, g, mesh.quaternion);
        }
    }
    
    // Add bodies to scene
    for (let b = 0; b < model.nbody; b++) {
        if (bodies[b]) {
            if (b === 0 || !bodies[0]) {
                mujocoRoot.add(bodies[b]);
            } else {
                bodies[0].add(bodies[b]);
            }
        }
    }
    
    // Update initial transforms
    updateTransforms();
    
    console.log('Scene built');
}

// Create geometry from MuJoCo type
function createGeometry(mujoco, type, size) {
    if (type === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
        return new THREE.SphereGeometry(size[0], 20, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
        return new THREE.CapsuleGeometry(size[0], size[1] * 2, 20, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
        return new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_BOX.value) {
        return new THREE.BoxGeometry(size[0] * 2, size[2] * 2, size[1] * 2);
    }
    
    return new THREE.SphereGeometry(size[0]);
}

// Create material
function createMaterial(model, geomIndex) {
    const color = [
        model.geom_rgba[geomIndex * 4 + 0],
        model.geom_rgba[geomIndex * 4 + 1],
        model.geom_rgba[geomIndex * 4 + 2],
        model.geom_rgba[geomIndex * 4 + 3]
    ];
    
    return new THREE.MeshPhysicalMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        transparent: color[3] < 1.0,
        opacity: color[3],
        metalness: 0.3,
        roughness: 0.6,
    });
}

// Get position with coordinate swizzle (Z-up to Y-up)
function getPosition(buffer, index, target) {
    target.set(
        buffer[index * 3 + 0],
        buffer[index * 3 + 2],
        -buffer[index * 3 + 1]
    );
}

// Get quaternion with coordinate swizzle
function getQuaternion(buffer, index, target) {
    target.set(
        -buffer[index * 4 + 1],
        -buffer[index * 4 + 3],
        buffer[index * 4 + 2],
        -buffer[index * 4 + 0]
    );
}

// Update body transforms from simulation state
function updateTransforms() {
    for (let b = 0; b < model.nbody; b++) {
        if (bodies[b]) {
            getPosition(data.xpos, b, bodies[b].position);
            getQuaternion(data.xquat, b, bodies[b].quaternion);
        }
    }
}

// Reset simulation
function reset() {
    if (!mujoco || !model || !data) return;
    mujoco.mj_resetData(model, data);
    updateTransforms();
}

// Step simulation once
function step() {
    if (!mujoco || !model || !data) return;
    mujoco.mj_step(model, data);
    updateTransforms();
}

// Toggle play/pause
function togglePlayPause() {
    isPlaying = !isPlaying;
    playPauseBtn.textContent = isPlaying ? 'Pause' : 'Play';
    stepBtn.disabled = isPlaying;
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    
    if (isPlaying && mujoco && model && data) {
        mujoco.mj_step(model, data);
        updateTransforms();
        
        // FPS counter
        frameCount++;
        const now = performance.now();
        if (now - lastTime >= 1000) {
            statsSpan.textContent = `${frameCount} FPS`;
            frameCount = 0;
            lastTime = now;
        }
    }
    
    controls.update();
    renderer.render(scene, camera);
}

// Handle resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Controls
playPauseBtn.addEventListener('click', togglePlayPause);
resetBtn.addEventListener('click', reset);
stepBtn.addEventListener('click', step);

// Keyboard
window.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
        e.preventDefault();
        togglePlayPause();
    } else if (e.code === 'KeyR') {
        reset();
    } else if (e.code === 'KeyS' && !isPlaying) {
        step();
    }
});

// Initialize
(async () => {
    try {
        await loadMuJoCo();
        const manifest = await downloadFiles();
        buildScene();
        
        loadingDiv.style.display = 'none';
        uiDiv.style.display = 'block';
        statusSpan.textContent = `${model.nbody} bodies`;
        
        animate();
    } catch (error) {
        console.error('Failed to initialize:', error);
        loadingDiv.querySelector('div:last-child').textContent = 'Error: ' + error.message;
    }
})();
