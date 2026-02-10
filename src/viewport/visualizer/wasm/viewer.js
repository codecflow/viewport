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
        const geometry = createGeometry(mujoco, type, size, g);
        const material = createMaterial(mujoco, model, g);
        
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

// Mesh cache
const meshes = {};

// Create geometry from MuJoCo type
function createGeometry(mujoco, type, size, geomIndex) {
    if (type === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
        return new THREE.SphereGeometry(size[0], 20, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
        return new THREE.CapsuleGeometry(size[0], size[1] * 2, 20, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
        return new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_BOX.value) {
        return new THREE.BoxGeometry(size[0] * 2, size[2] * 2, size[1] * 2);
    } else if (type === mujoco.mjtGeom.mjGEOM_MESH.value) {
        const meshID = model.geom_dataid[geomIndex];
        
        if (!(meshID in meshes)) {
            const geometry = new THREE.BufferGeometry();
            
            // Extract vertex data
            const vertexBuffer = model.mesh_vert.subarray(
                model.mesh_vertadr[meshID] * 3,
                (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3
            );
            // Swizzle Y/Z coordinates
            for (let v = 0; v < vertexBuffer.length; v += 3) {
                const temp = vertexBuffer[v + 1];
                vertexBuffer[v + 1] = vertexBuffer[v + 2];
                vertexBuffer[v + 2] = -temp;
            }
            
            // Extract normal data
            const normalBuffer = model.mesh_normal.subarray(
                model.mesh_normaladr[meshID] * 3,
                (model.mesh_normaladr[meshID] + model.mesh_normalnum[meshID]) * 3
            );
            // Swizzle normals
            for (let v = 0; v < normalBuffer.length; v += 3) {
                const temp = normalBuffer[v + 1];
                normalBuffer[v + 1] = normalBuffer[v + 2];
                normalBuffer[v + 2] = -temp;
            }
            
            // Extract UV data
            const uvBuffer = model.mesh_texcoord.subarray(
                model.mesh_texcoordadr[meshID] * 2,
                (model.mesh_texcoordadr[meshID] + model.mesh_texcoordnum[meshID]) * 2
            );
            
            // Extract face indices
            const triangleBuffer = model.mesh_face.subarray(
                model.mesh_faceadr[meshID] * 3,
                (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3
            );
            
            // Build indexed geometry with proper UV/normal mapping
            const positions = [];
            const normals = [];
            const uvs = [];
            const indices = [];
            const tupleToIndex = new Map();
            
            const faceToUvBuffer = model.mesh_facetexcoord?.subarray(
                model.mesh_faceadr[meshID] * 3,
                (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3
            );
            const faceToNormalBuffer = model.mesh_facenormal?.subarray(
                model.mesh_faceadr[meshID] * 3,
                (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3
            );
            
            const faceCount = triangleBuffer.length / 3;
            for (let t = 0; t < faceCount; t++) {
                for (let c = 0; c < 3; c++) {
                    const vi = triangleBuffer[t * 3 + c];
                    const nvi = faceToNormalBuffer ? faceToNormalBuffer[t * 3 + c] : vi;
                    const uvi = faceToUvBuffer ? faceToUvBuffer[t * 3 + c] : vi;
                    const key = `${vi}_${nvi}_${uvi}`;
                    
                    let outIndex = tupleToIndex.get(key);
                    if (outIndex === undefined) {
                        outIndex = positions.length / 3;
                        tupleToIndex.set(key, outIndex);
                        
                        positions.push(
                            vertexBuffer[vi * 3 + 0],
                            vertexBuffer[vi * 3 + 1],
                            vertexBuffer[vi * 3 + 2]
                        );
                        normals.push(
                            normalBuffer[nvi * 3 + 0],
                            normalBuffer[nvi * 3 + 1],
                            normalBuffer[nvi * 3 + 2]
                        );
                        if (uvBuffer.length > 0) {
                            uvs.push(uvBuffer[uvi * 2 + 0], uvBuffer[uvi * 2 + 1]);
                        }
                    }
                    indices.push(outIndex);
                }
            }
            
            geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
            geometry.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
            if (uvs.length > 0) {
                geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
            }
            geometry.setIndex(indices);
            
            meshes[meshID] = geometry;
        }
        
        return meshes[meshID];
    }
    
    return new THREE.SphereGeometry(size[0]);
}

// Texture cache
const textures = {};

// Create texture from model data
function createTexture(mujoco, model, texId) {
    if (texId in textures) {
        return textures[texId];
    }
    
    const width = model.tex_width[texId];
    const height = model.tex_height[texId];
    const offset = model.tex_adr[texId];
    const channels = model.tex_nchannel[texId];
    
    // Convert to RGBA
    const rgbaArray = new Uint8Array(width * height * 4);
    for (let p = 0; p < width * height; p++) {
        rgbaArray[p * 4 + 0] = model.tex_data[offset + p * channels + 0]; // R
        rgbaArray[p * 4 + 1] = channels > 1 ? model.tex_data[offset + p * channels + 1] : rgbaArray[p * 4 + 0]; // G
        rgbaArray[p * 4 + 2] = channels > 2 ? model.tex_data[offset + p * channels + 2] : rgbaArray[p * 4 + 0]; // B
        rgbaArray[p * 4 + 3] = channels > 3 ? model.tex_data[offset + p * channels + 3] : 255; // A
    }
    
    const texture = new THREE.DataTexture(
        rgbaArray,
        width,
        height,
        THREE.RGBAFormat,
        THREE.UnsignedByteType
    );
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.needsUpdate = true;
    
    textures[texId] = texture;
    return texture;
}

// Create material
function createMaterial(mujoco, model, geomIndex) {
    let color = [
        model.geom_rgba[geomIndex * 4 + 0],
        model.geom_rgba[geomIndex * 4 + 1],
        model.geom_rgba[geomIndex * 4 + 2],
        model.geom_rgba[geomIndex * 4 + 3]
    ];
    
    let texture = null;
    let texRepeat = [1, 1];
    
    // Check if geom has material
    if (model.geom_matid[geomIndex] !== -1) {
        const matId = model.geom_matid[geomIndex];
        
        // Use material RGBA
        color = [
            model.mat_rgba[matId * 4 + 0],
            model.mat_rgba[matId * 4 + 1],
            model.mat_rgba[matId * 4 + 2],
            model.mat_rgba[matId * 4 + 3]
        ];
        
        // Check for texture
        if (model.mat_texid && model.ntex > 0) {
            const mjNTEXROLE = 10;
            const mjTEXROLE_RGB = 1;
            const texId = model.mat_texid[matId * mjNTEXROLE + mjTEXROLE_RGB];
            
            if (texId !== -1) {
                texture = createTexture(mujoco, model, texId);
                
                if (model.mat_texrepeat) {
                    texRepeat = [
                        model.mat_texrepeat[matId * 2 + 0],
                        model.mat_texrepeat[matId * 2 + 1]
                    ];
                }
            }
        }
    }
    
    // PBR material properties based on color
    const isDark = color[0] < 0.3 && color[1] < 0.3 && color[2] < 0.3;
    const isLight = color[0] > 0.6 || color[1] > 0.6 || color[2] > 0.6;
    
    const material = new THREE.MeshPhysicalMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        transparent: color[3] < 1.0,
        opacity: color[3],
        metalness: isDark ? 0.8 : 0.3,
        roughness: isDark ? 0.4 : 0.6,
        clearcoat: isLight ? 0.3 : 0.0,
        clearcoatRoughness: 0.4,
    });
    
    if (texture) {
        material.map = texture;
        texture.repeat.set(texRepeat[0], texRepeat[1]);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
    }
    
    return material;
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
