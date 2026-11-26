use thiserror::Error;

use crate::device::{create_instance, request_device};
use crate::readback::{ReadbackError, TextureReadback};
use crate::renderer::{
    create_depth_view, validate_scene, GpuSceneRenderer, RenderSize, RenderTarget,
};
use crate::scene::SceneData;
use crate::view::ViewSpec;

#[derive(Debug, Error)]
pub enum RenderError {
    #[error("no wgpu adapter available (no GPU / software rasterizer installed)")]
    NoAdapter,
    #[error("failed to create the wgpu device: {0}")]
    Device(String),
    #[error("scene has no geometry to draw")]
    EmptyScene,
    #[error("scene declares {num_cameras} cameras but the packed vertex layout holds at most {max}")]
    TooManyCameras { num_cameras: usize, max: usize },
    #[error("scene declares {num_cameras} cameras but carries {images} camera images")]
    CameraImageCountMismatch { num_cameras: usize, images: usize },
    #[error("render target must be non-empty, got {width}x{height}")]
    InvalidSize { width: u32, height: u32 },
    #[error("camera image {index} is {width}x{height}, expected {expected_width}x{expected_height}")]
    ImageSizeMismatch {
        index: usize,
        width: u32,
        height: u32,
        expected_width: u32,
        expected_height: u32,
    },
    #[error("failed to read back the rendered frame: {0}")]
    Readback(String),
}

impl From<ReadbackError> for RenderError {
    fn from(error: ReadbackError) -> Self {
        Self::Readback(error.to_string())
    }
}

const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

/// Render every view offscreen with an opaque black background.
///
/// Returns [`RenderError::NoAdapter`] (never panics) when no GPU adapter is
/// available.
pub fn render_all_views(
    scene: &SceneData,
    views: &[ViewSpec],
    size: RenderSize,
) -> Result<Vec<Vec<u8>>, RenderError> {
    render_all_views_with(scene, views, size, [0.0, 0.0, 0.0, 1.0])
}

/// Render every view offscreen and return one RGBA8 frame per view, clearing to
/// `clear_color` (linear RGBA in `[0, 1]`, matching `wgpu::Color`).
pub fn render_all_views_with(
    scene: &SceneData,
    views: &[ViewSpec],
    size: RenderSize,
    clear_color: [f64; 4],
) -> Result<Vec<Vec<u8>>, RenderError> {
    validate_scene(scene)?;
    if size.width == 0 || size.height == 0 {
        return Err(RenderError::InvalidSize {
            width: size.width,
            height: size.height,
        });
    }

    let (device, queue) = create_offscreen_device()?;
    let renderer = GpuSceneRenderer::new(&device, &queue, scene, COLOR_FORMAT)?;
    let frame = OffscreenFrame {
        device: &device,
        queue: &queue,
        renderer: &renderer,
        targets: OffscreenTargets::new(&device, size),
        clear: wgpu::Color {
            r: clear_color[0],
            g: clear_color[1],
            b: clear_color[2],
            a: clear_color[3],
        },
    };

    views.iter().map(|view| frame.render_view(view)).collect()
}

fn create_offscreen_device() -> Result<(wgpu::Device, wgpu::Queue), RenderError> {
    let instance = create_instance();
    let (_adapter, device, queue) = request_device(&instance, None, "avm-offscreen-device")?;
    Ok((device, queue))
}

/// Color + depth attachments shared by every view of one offscreen render.
struct OffscreenTargets {
    color_texture: wgpu::Texture,
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    size: RenderSize,
}

impl OffscreenTargets {
    fn new(device: &wgpu::Device, size: RenderSize) -> Self {
        let color_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("avm-offscreen-color"),
            size: wgpu::Extent3d {
                width: size.width,
                height: size.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_view = create_depth_view(device, size);
        Self {
            color_texture,
            color_view,
            depth_view,
            size,
        }
    }
}

/// One device + one set of targets + one clear colour: everything a single view
/// needs besides the view itself.
struct OffscreenFrame<'a> {
    device: &'a wgpu::Device,
    queue: &'a wgpu::Queue,
    renderer: &'a GpuSceneRenderer,
    targets: OffscreenTargets,
    clear: wgpu::Color,
}

impl OffscreenFrame<'_> {
    fn render_view(&self, view: &ViewSpec) -> Result<Vec<u8>, RenderError> {
        let readback = TextureReadback::new(self.device, self.targets.size);
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("avm-offscreen-encoder"),
            });
        self.renderer.render(
            self.queue,
            &mut encoder,
            RenderTarget {
                color: &self.targets.color_view,
                depth: &self.targets.depth_view,
            },
            view,
            self.clear,
        );
        readback.copy_from_texture(&mut encoder, &self.targets.color_texture);
        self.queue.submit([encoder.finish()]);
        readback
            .read(self.device, COLOR_FORMAT)
            .map_err(RenderError::from)
    }
}
