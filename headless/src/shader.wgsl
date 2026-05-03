// Uniforms: view-projection matrix + lighting
struct Uniforms {
    view_proj: mat4x4<f32>,
    light_dir: vec3<f32>,
    _pad0: f32,
    light_color: vec3<f32>,
    ambient: f32,
}

@group(0) @binding(0)
var<uniform> u: Uniforms;

// Vertex input/output
struct VIn {
    @location(0) position: vec3<f32>,
    @location(1) normal:   vec3<f32>,
    @location(2) color:    vec4<f32>,
}

struct VOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec4<f32>,
}

@vertex
fn vs_main(in: VIn) -> VOut {
    var out: VOut;
    out.clip_pos    = u.view_proj * vec4<f32>(in.position, 1.0);
    out.world_normal = in.normal;
    out.color        = in.color;
    return out;
}

@fragment
fn fs_main(in: VOut) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let diffuse = max(dot(n, normalize(u.light_dir)), 0.0);
    let light = u.light_color * diffuse + u.ambient;
    return vec4<f32>(in.color.rgb * light, in.color.a);
}
