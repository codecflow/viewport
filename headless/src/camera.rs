//! Perspective camera producing a view-projection matrix for wgpu (Z in [0,1]).

use glam::{Mat4, Vec3};

pub struct Camera {
    pub eye: Vec3,
    pub center: Vec3,
    pub up: Vec3,
    pub fov_y_deg: f32,
    pub aspect: f32,
    pub near: f32,
    pub far: f32,
}

impl Camera {
    /// Right-handed perspective projection with depth range [0, 1] (wgpu convention).
    pub fn view_proj(&self) -> Mat4 {
        let view = Mat4::look_at_rh(self.eye, self.center, self.up);
        let proj = Mat4::perspective_rh(self.fov_y_deg.to_radians(), self.aspect, self.near, self.far);
        proj * view
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            eye: Vec3::new(3.0, -3.0, 3.0),
            center: Vec3::new(0.0, 0.0, 0.5),
            up: Vec3::new(0.0, 0.0, 1.0),
            fov_y_deg: 60.0,
            aspect: 1.0,
            near: 0.05,
            far: 200.0,
        }
    }
}
