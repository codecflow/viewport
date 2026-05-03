//! Vertex generation for VisualScene primitive geometry types.

use std::f32::consts::PI;

use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3, Vec4};

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable, Debug)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}

pub fn transform_vertex(v: Vertex, mat: Mat4) -> Vertex {
    let pos4 = mat * Vec4::new(v.position[0], v.position[1], v.position[2], 1.0);
    let n = mat.transform_vector3(Vec3::from(v.normal)).normalize_or_zero();
    Vertex { position: [pos4.x, pos4.y, pos4.z], normal: n.into(), color: v.color }
}

/// Box centered at origin. size = [w, h, d] (full extents → stored as half).
pub fn make_box(w: f32, h: f32, d: f32, color: [f32; 4]) -> Vec<Vertex> {
    let (hx, hy, hz) = (w * 0.5, h * 0.5, d * 0.5);
    // face: [quad vertices], normal
    type Quad = [(f32, f32, f32); 4];
    let faces: [(Quad, [f32; 3]); 6] = [
        ([(0.0-hx,0.0-hy, hz), ( hx,0.0-hy, hz), ( hx, hy, hz),(0.0-hx, hy, hz)], [ 0.0, 0.0, 1.0]),
        ([(0.0-hx,  hy,0.0-hz), ( hx,  hy,0.0-hz), ( hx,0.0-hy,0.0-hz),(0.0-hx,0.0-hy,0.0-hz)], [ 0.0, 0.0,-1.0]),
        ([(  hx,0.0-hy,0.0-hz), ( hx,  hy,0.0-hz), ( hx,  hy,  hz),(  hx,0.0-hy,  hz)], [ 1.0, 0.0, 0.0]),
        ([(0.0-hx,0.0-hy,  hz),(0.0-hx,  hy,  hz),(0.0-hx,  hy,0.0-hz),(0.0-hx,0.0-hy,0.0-hz)], [-1.0, 0.0, 0.0]),
        ([(0.0-hx,  hy,0.0-hz), ( hx,  hy,0.0-hz), ( hx,  hy,  hz),(0.0-hx,  hy,  hz)], [ 0.0, 1.0, 0.0]),
        ([(0.0-hx,0.0-hy,  hz), ( hx,0.0-hy,  hz), ( hx,0.0-hy,0.0-hz),(0.0-hx,0.0-hy,0.0-hz)], [ 0.0,-1.0, 0.0]),
    ];
    let mut verts = Vec::with_capacity(36);
    for (quad, normal) in &faces {
        let v = |i: usize| { let (px,py,pz) = quad[i]; Vertex { position: [px,py,pz], normal: *normal, color } };
        verts.extend_from_slice(&[v(0),v(1),v(2), v(0),v(2),v(3)]);
    }
    verts
}

/// UV sphere. size = [radius].
pub fn make_sphere(radius: f32, color: [f32; 4]) -> Vec<Vertex> {
    let (stacks, sectors) = (12u32, 16u32);
    let stack_step = PI / stacks as f32;
    let sector_step = 2.0 * PI / sectors as f32;

    let vert = |stack: u32, sector: u32| {
        let phi = PI * 0.5 - stack as f32 * stack_step;
        let theta = sector as f32 * sector_step;
        let (cp, sp) = (phi.cos(), phi.sin());
        let (ct, st) = (theta.cos(), theta.sin());
        Vertex { position: [radius*cp*ct, radius*cp*st, radius*sp], normal: [cp*ct, cp*st, sp], color }
    };

    let mut verts = Vec::with_capacity((stacks * sectors * 6) as usize);
    for i in 0..stacks {
        for j in 0..sectors {
            let (v0,v1,v2,v3) = (vert(i,j), vert(i+1,j), vert(i+1,j+1), vert(i,j+1));
            verts.extend_from_slice(&[v0,v1,v2, v0,v2,v3]);
        }
    }
    verts
}

/// Cylinder (axis = Z). size = [radius, height].
pub fn make_cylinder(radius: f32, height: f32, color: [f32; 4]) -> Vec<Vertex> {
    let sectors = 16u32;
    let step = 2.0 * PI / sectors as f32;
    let hz = height * 0.5;
    let mut verts = Vec::new();

    for j in 0..sectors {
        let (t0, t1) = (j as f32 * step, (j+1) as f32 * step);
        let (c0, s0, c1, s1) = (t0.cos(), t0.sin(), t1.cos(), t1.sin());
        let sv = |c: f32, s: f32, z: f32| Vertex { position: [radius*c, radius*s, z], normal: [c,s,0.0], color };
        let (a,b,c,d) = (sv(c0,s0,0.0-hz), sv(c1,s1,0.0-hz), sv(c1,s1,hz), sv(c0,s0,hz));
        verts.extend_from_slice(&[a,b,c, a,c,d]);
    }
    // Caps: top (+Z) and bottom (-Z)
    for (sign, nz, ccw) in [(1.0f32, 1.0f32, true), (0.0-1.0, 0.0-1.0, false)] {
        let center = Vertex { position: [0.0,0.0,sign*hz], normal: [0.0,0.0,nz], color };
        for j in 0..sectors {
            let (t0,t1) = (j as f32 * step, (j+1) as f32 * step);
            let (r0,r1) = (
                Vertex { position: [radius*t0.cos(), radius*t0.sin(), sign*hz], normal: [0.0,0.0,nz], color },
                Vertex { position: [radius*t1.cos(), radius*t1.sin(), sign*hz], normal: [0.0,0.0,nz], color },
            );
            if ccw { verts.extend_from_slice(&[center,r0,r1]); }
            else   { verts.extend_from_slice(&[center,r1,r0]); }
        }
    }
    verts
}

/// Capsule (axis = Z). size = [radius, cylinder_height].
pub fn make_capsule(radius: f32, cyl_height: f32, color: [f32; 4]) -> Vec<Vertex> {
    let mut verts = make_cylinder(radius, cyl_height, color);
    let hz = cyl_height * 0.5;
    let (stacks, sectors) = (6u32, 16u32);
    let sector_step = 2.0 * PI / sectors as f32;
    let stack_step = (PI * 0.5) / stacks as f32;

    for (offset_z, flip) in [(hz, false), (0.0-hz, true)] {
        for i in 0..stacks {
            for j in 0..sectors {
                let make_v = |stack: u32, sector: u32| {
                    let phi = if flip { 0.0-(stack as f32 * stack_step) } else { stack as f32 * stack_step };
                    let theta = sector as f32 * sector_step;
                    let (cp, sp) = (phi.cos(), phi.sin());
                    let (ct, st) = (theta.cos(), theta.sin());
                    Vertex { position: [radius*cp*ct, radius*cp*st, offset_z + radius*sp], normal: [cp*ct,cp*st,sp], color }
                };
                let (v0,v1,v2,v3) = (make_v(i,j), make_v(i,j+1), make_v(i+1,j+1), make_v(i+1,j));
                verts.extend_from_slice(&[v0,v1,v2, v0,v2,v3]);
            }
        }
    }
    verts
}

/// Ground plane at Z=0. size = [half_w, half_d].
pub fn make_plane(half_w: f32, half_d: f32, color: [f32; 4]) -> Vec<Vertex> {
    let n = [0.0f32, 0.0, 1.0];
    let hw = half_w;
    let hd = half_d;
    vec![
        Vertex { position: [0.0-hw, 0.0-hd, 0.0], normal: n, color },
        Vertex { position: [    hw, 0.0-hd, 0.0], normal: n, color },
        Vertex { position: [    hw,     hd, 0.0], normal: n, color },
        Vertex { position: [0.0-hw, 0.0-hd, 0.0], normal: n, color },
        Vertex { position: [    hw,     hd, 0.0], normal: n, color },
        Vertex { position: [0.0-hw,     hd, 0.0], normal: n, color },
    ]
}
