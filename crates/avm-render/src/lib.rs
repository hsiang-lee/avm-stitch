pub mod device;
pub mod image_source;
pub mod offscreen;
pub mod readback;
pub mod renderer;
pub mod scene;
pub mod shader;
pub mod vertex;
pub mod view;

pub use device::{create_instance, request_device};
pub use image_source::{load_camera_image, ImageError};
pub use offscreen::{render_all_views, render_all_views_with, RenderError};
pub use readback::{ReadbackError, TextureReadback};
pub use renderer::{
    create_depth_view, GpuSceneRenderer, RenderSize, RenderTarget, CAMERA_FORMAT, DEPTH_FORMAT,
};
pub use scene::SceneData;
pub use shader::SHADER_SOURCE;
pub use vertex::{
    PackedVertex, VertexAttributes, NUM_UV_SLOTS, NUM_WEIGHT_SLOTS, PACKED_VERTEX_SIZE,
};
pub use view::ViewSpec;

/// Background behind the bowl, RGB(30, 36, 50) with an opaque alpha. The parity
/// fixtures were captured with exactly this value.
pub const BACKGROUND_CLEAR: [f64; 4] = [30.0 / 255.0, 36.0 / 255.0, 50.0 / 255.0, 1.0];
