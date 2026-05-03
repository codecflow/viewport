//! VisualScene types — mirrors client/src/viewer/types.ts.
//! Only what the headless renderer needs (rigid bodies + lights).

use serde::Deserialize;
use std::collections::HashMap;

#[derive(Debug, Clone, Deserialize)]
pub struct VisualScene {
    pub bodies: Vec<BodyDesc>,
    pub lights: Vec<LightDesc>,
    #[serde(default)]
    pub materials: HashMap<String, MaterialDesc>,
    #[serde(default)]
    pub meshes: HashMap<String, String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum BodyDesc {
    Rigid(RigidBodyDesc),
    Soft(SoftBodyDesc),
    Skinned { name: String },
    Instanced { name: String },
}

#[derive(Debug, Clone, Deserialize)]
pub struct RigidBodyDesc {
    pub name: String,
    pub pos: [f32; 3],
    #[serde(default)]
    pub rot: Option<[f32; 3]>,
    #[serde(default)]
    pub quat: Option<[f32; 4]>, // wxyz
    pub geoms: Vec<GeomDesc>,
    #[serde(default)]
    pub parent: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SoftBodyDesc {
    pub name: String,
    #[serde(rename = "vertexCount")]
    pub vertex_count: usize,
}

#[derive(Debug, Clone, Deserialize)]
pub struct GeomDesc {
    #[serde(rename = "type")]
    pub geom_type: String,
    pub size: Vec<f32>,
    #[serde(default)]
    pub pos: Option<[f32; 3]>,
    #[serde(default)]
    pub rot: Option<[f32; 3]>,
    #[serde(default)]
    pub quat: Option<[f32; 4]>, // wxyz
    #[serde(default)]
    pub material: Option<MaterialOrRef>,
    #[serde(default)]
    pub mesh: Option<String>,
    #[serde(rename = "meshUrl", default)]
    pub mesh_url: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum MaterialOrRef {
    Ref(String),
    Inline(MaterialDesc),
}

#[derive(Debug, Clone, Deserialize)]
pub struct MaterialDesc {
    #[serde(default)]
    pub color: Option<[f32; 4]>, // RGBA 0-1
    #[serde(default)]
    pub roughness: Option<f32>,
    #[serde(default)]
    pub metalness: Option<f32>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LightDesc {
    #[serde(rename = "type")]
    pub light_type: String,
    pub name: String,
    #[serde(default)]
    pub pos: Option<[f32; 3]>,
    #[serde(default)]
    pub color: Option<[f32; 3]>, // RGB 0-1
    #[serde(default)]
    pub intensity: Option<f32>,
    #[serde(default)]
    pub dir: Option<[f32; 3]>,
}

impl MaterialDesc {
    pub fn color_or_default(&self) -> [f32; 4] {
        self.color.unwrap_or([0.42, 0.45, 0.5, 1.0])
    }
}
