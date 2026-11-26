use avm_core::bowl::BowlModel;

use crate::vertex::PackedVertex;

pub struct SceneData {
    pub vertices: Vec<PackedVertex>,
    pub indices: Vec<u32>,
    pub num_cameras: usize,
    /// RGBA8 layers for the camera texture array (index-aligned with cameras).
    pub camera_images: Vec<image::RgbaImage>,
}

impl SceneData {
    pub fn from_bowl_model(model: &BowlModel) -> Self {
        Self {
            vertices: PackedVertex::vertex_buffer(model),
            indices: PackedVertex::index_buffer(model),
            num_cameras: model.baked_num_cameras(),
            camera_images: Vec::new(),
        }
    }

    pub fn with_camera_images(mut self, images: Vec<image::RgbaImage>) -> Self {
        self.camera_images = images;
        self
    }
}
