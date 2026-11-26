//! GPU side of the avm-stitch Rust port (wgpu + WGSL).
//!
//! The C++ reference implementation lives in `src/render/stitch_scene.cpp`
//! (OSG + GLSL 330). This crate ports the vertex packing and the fragment
//! sampling logic, plus an offscreen render entry point.

pub mod image_source;
pub mod offscreen;
pub mod scene;
pub mod shader;
pub mod vertex;
pub mod view;

pub use image_source::{load_camera_image, ImageError};
pub use offscreen::{render_all_views, RenderError};
pub use scene::SceneData;
pub use shader::SHADER_SOURCE;
pub use vertex::{
    PackedVertex, VertexAttributes, NUM_UV_SLOTS, NUM_WEIGHT_SLOTS, PACKED_VERTEX_SIZE,
};
pub use view::ViewSpec;
