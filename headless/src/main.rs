//! simarena-headless — render a VisualScene JSON to a PNG image using wgpu.
//!
//! Usage:
//!   cargo run -- --scene scene.json --output frame.png
//!   cargo run -- --scene scene.json --output frame.png --width 1280 --height 720
//!   curl http://localhost:8080/scene | cargo run -- --output frame.png

mod camera;
mod primitive;
mod renderer;
mod types;

use std::path::PathBuf;

use camera::Camera;
use clap::Parser;
use renderer::Renderer;
use types::VisualScene;

#[derive(Parser, Debug)]
#[command(name = "simarena-headless", about = "Render a VisualScene JSON to a PNG")]
struct Args {
    /// Path to VisualScene JSON (or omit to read from stdin)
    #[arg(short, long)]
    scene: Option<PathBuf>,

    /// Output PNG path
    #[arg(short, long, default_value = "frame.png")]
    output: PathBuf,

    /// Image width in pixels
    #[arg(long, default_value_t = 960)]
    width: u32,

    /// Image height in pixels
    #[arg(long, default_value_t = 540)]
    height: u32,

    /// Camera eye X
    #[arg(long, default_value_t = 3.0)]
    cam_x: f32,

    /// Camera eye Y
    #[arg(long, default_value_t = -3.0)]
    cam_y: f32,

    /// Camera eye Z
    #[arg(long, default_value_t = 3.0)]
    cam_z: f32,
}

fn main() {
    let args = Args::parse();

    // Read VisualScene JSON
    let json = match &args.scene {
        Some(path) => std::fs::read_to_string(path)
            .unwrap_or_else(|e| { eprintln!("Failed to read {}: {e}", path.display()); std::process::exit(1); }),
        None => {
            use std::io::Read;
            let mut s = String::new();
            std::io::stdin().read_to_string(&mut s)
                .unwrap_or_else(|e| { eprintln!("Failed to read stdin: {e}"); std::process::exit(1); });
            s
        }
    };

    let scene: VisualScene = serde_json::from_str(&json)
        .unwrap_or_else(|e| { eprintln!("Invalid VisualScene JSON: {e}"); std::process::exit(1); });

    eprintln!("[headless] loaded {} bodies, {} lights", scene.bodies.len(), scene.lights.len());

    // Build renderer
    let renderer = pollster::block_on(Renderer::new(args.width, args.height));

    // Camera
    let camera = Camera {
        eye: glam::Vec3::new(args.cam_x, args.cam_y, args.cam_z),
        aspect: args.width as f32 / args.height as f32,
        ..Camera::default()
    };

    // Render
    let rgba = renderer.render(&scene, &camera);
    eprintln!("[headless] rendered {}×{}", args.width, args.height);

    // Save PNG
    image::save_buffer(&args.output, &rgba, args.width, args.height, image::ColorType::Rgba8)
        .unwrap_or_else(|e| { eprintln!("Failed to save PNG: {e}"); std::process::exit(1); });

    eprintln!("[headless] saved → {}", args.output.display());
}
