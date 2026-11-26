pub mod image_source;
pub mod offscreen;
pub mod renderer;
pub mod scene;
pub mod shader;
pub mod vertex;
pub mod view;

pub use image_source::{load_camera_image, ImageError};
pub use offscreen::{render_all_views, render_all_views_with, RenderError};
pub use renderer::{create_depth_view, GpuSceneRenderer, CAMERA_FORMAT, DEPTH_FORMAT};
pub use scene::SceneData;
pub use shader::SHADER_SOURCE;
pub use vertex::{
    PackedVertex, VertexAttributes, NUM_UV_SLOTS, NUM_WEIGHT_SLOTS, PACKED_VERTEX_SIZE,
};
pub use view::ViewSpec;
