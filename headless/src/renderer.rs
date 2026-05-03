//! Headless wgpu renderer: VisualScene → RGBA image bytes.

use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::camera::Camera;
use crate::primitive::{
    make_box, make_capsule, make_cylinder, make_plane, make_sphere, transform_vertex, Vertex,
};
use crate::types::{BodyDesc, MaterialOrRef, VisualScene};

/// Aligned bytes-per-row for output buffer copies.
fn aligned_bpr(width: u32) -> u32 {
    const COPY_ALIGN: u32 = 256;
    (width * 4 + COPY_ALIGN - 1) & !(COPY_ALIGN - 1)
}

/// Uniform buffer layout matching shader.wgsl.
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 3],
    _pad0: f32,
    light_color: [f32; 3],
    ambient: f32,
}

pub struct Renderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    uniform_bgl: wgpu::BindGroupLayout,
    pub width: u32,
    pub height: u32,
}

impl Renderer {
    pub async fn new(width: u32, height: u32) -> Self {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .expect("no wgpu adapter found");

        eprintln!("[headless] adapter: {}", adapter.get_info().name);

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("headless"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::downlevel_defaults(),
                    ..Default::default()
                },
                None,
            )
            .await
            .expect("device request failed");

        let shader = device.create_shader_module(wgpu::include_wgsl!("shader.wgsl"));

        let uniform_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("ub_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&uniform_bgl],
            push_constant_ranges: &[],
        });

        let stride = std::mem::size_of::<Vertex>() as u64;
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("main"),
            layout: Some(&pl),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: stride,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute { shader_location: 0, offset: 0,  format: wgpu::VertexFormat::Float32x3 },
                        wgpu::VertexAttribute { shader_location: 1, offset: 12, format: wgpu::VertexFormat::Float32x3 },
                        wgpu::VertexAttribute { shader_location: 2, offset: 24, format: wgpu::VertexFormat::Float32x4 },
                    ],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Rgba8UnormSrgb,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Renderer { device, queue, pipeline, uniform_bgl, width, height }
    }

    /// Render VisualScene with the given camera. Returns raw RGBA bytes (width × height × 4).
    pub fn render(&self, scene: &VisualScene, camera: &Camera) -> Vec<u8> {
        let verts = self.build_vertices(scene);

        if verts.is_empty() {
            let bg: u8 = 0x1a;
            return vec![bg, bg, bg, 0xff].into_iter().cycle().take((self.width * self.height * 4) as usize).collect();
        }

        // Uniform buffer
        let (light_dir, light_color, ambient) = Self::extract_lighting(scene);
        let uni = Uniforms {
            view_proj: camera.view_proj().to_cols_array_2d(),
            light_dir,
            _pad0: 0.0,
            light_color,
            ambient,
        };
        let ubuf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("uniforms"),
            contents: bytemuck::bytes_of(&uni),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let bg = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &self.uniform_bgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: ubuf.as_entire_binding() }],
        });

        // Vertex buffer
        let vbuf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("verts"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Color + depth textures
        let size = wgpu::Extent3d { width: self.width, height: self.height, depth_or_array_layers: 1 };
        let color_tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("color"), size, mip_level_count: 1, sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth_tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"), size, mip_level_count: 1, sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });

        // Output buffer (aligned rows)
        let bpr = aligned_bpr(self.width);
        let out_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("out"),
            size: (bpr * self.height) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let mut enc = self.device.create_command_encoder(&Default::default());
        {
            let cv = color_tex.create_view(&Default::default());
            let dv = depth_tex.create_view(&Default::default());
            let mut rp = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &cv,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.1, g: 0.11, b: 0.12, a: 1.0 }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &dv,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            rp.set_pipeline(&self.pipeline);
            rp.set_bind_group(0, &bg, &[]);
            rp.set_vertex_buffer(0, vbuf.slice(..));
            rp.draw(0..verts.len() as u32, 0..1);
        }

        enc.copy_texture_to_buffer(
            color_tex.as_image_copy(),
            wgpu::ImageCopyBuffer {
                buffer: &out_buf,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(bpr),
                    rows_per_image: Some(self.height),
                },
            },
            size,
        );
        self.queue.submit([enc.finish()]);

        // Read back
        let slice = out_buf.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        self.device.poll(wgpu::Maintain::Wait);

        let data = slice.get_mapped_range();
        let mut rgba = Vec::with_capacity((self.width * self.height * 4) as usize);
        for row in 0..self.height {
            let start = (row * bpr) as usize;
            rgba.extend_from_slice(&data[start..start + (self.width * 4) as usize]);
        }
        rgba
    }

    // ── Private ──────────────────────────────────────────────────────────────

    fn build_vertices(&self, scene: &VisualScene) -> Vec<Vertex> {
        let mut all: Vec<Vertex> = Vec::new();

        for body in &scene.bodies {
            let BodyDesc::Rigid(body) = body else { continue };

            let body_pos = glam::Vec3::from(body.pos);
            let body_quat = body.quat
                .map(|q| glam::Quat::from_xyzw(q[1], q[2], q[3], q[0]))
                .unwrap_or(glam::Quat::IDENTITY);
            let body_mat = glam::Mat4::from_rotation_translation(body_quat, body_pos);

            for geom in &body.geoms {
                let geom_pos = geom.pos.map(glam::Vec3::from).unwrap_or(glam::Vec3::ZERO);
                let geom_quat = geom.quat
                    .map(|q| glam::Quat::from_xyzw(q[1], q[2], q[3], q[0]))
                    .unwrap_or(glam::Quat::IDENTITY);
                let world_mat = body_mat * glam::Mat4::from_rotation_translation(geom_quat, geom_pos);

                let color = Self::resolve_color(geom, scene);
                let s = &geom.size;
                let g0 = s.first().copied().unwrap_or(0.5);
                let g1 = s.get(1).copied().unwrap_or(0.5);
                let g2 = s.get(2).copied().unwrap_or(0.5);

                let verts: Vec<Vertex> = match geom.geom_type.as_str() {
                    "box"      => make_box(g0, g1, g2, color),
                    "sphere"   => make_sphere(g0, color),
                    "cylinder" => make_cylinder(g0, g1, color),
                    "capsule"  => make_capsule(g0, g1, color),
                    "plane"    => make_plane(g0, g1, color),
                    _ => continue,
                };

                all.extend(verts.iter().map(|&v| transform_vertex(v, world_mat)));
            }
        }
        all
    }

    fn resolve_color(geom: &crate::types::GeomDesc, scene: &VisualScene) -> [f32; 4] {
        let mat = match &geom.material {
            Some(MaterialOrRef::Inline(m)) => Some(m.clone()),
            Some(MaterialOrRef::Ref(name)) => scene.materials.get(name).cloned(),
            None => None,
        };
        mat.and_then(|m| m.color).unwrap_or([0.42, 0.45, 0.5, 1.0])
    }

    fn extract_lighting(scene: &VisualScene) -> ([f32; 3], [f32; 3], f32) {
        for l in &scene.lights {
            if l.light_type == "directional" {
                let d = l.dir.unwrap_or([0.5, 0.7, 1.0]);
                // negate: dir points from light; shader expects direction TO light
                let neg = [0.0 - d[0], 0.0 - d[1], 0.0 - d[2]];
                let c = l.color.unwrap_or([1.0, 1.0, 1.0]);
                let i = l.intensity.unwrap_or(1.0);
                return (neg, [c[0] * i, c[1] * i, c[2] * i], 0.15);
            }
        }
        ([0.5, 0.8, 1.0], [0.9, 0.9, 0.9], 0.15)
    }
}
