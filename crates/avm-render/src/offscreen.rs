use thiserror::Error;

use crate::renderer::{create_depth_view, validate_scene, GpuSceneRenderer};
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

const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

/// Render every view offscreen with an opaque black background.
///
/// Returns [`RenderError::NoAdapter`] (never panics) when no GPU adapter is
/// available.
pub fn render_all_views(
    scene: &SceneData,
    views: &[ViewSpec],
    width: u32,
    height: u32,
) -> Result<Vec<Vec<u8>>, RenderError> {
    render_all_views_with(scene, views, width, height, [0.0, 0.0, 0.0, 1.0])
}

/// Render every view offscreen and return one RGBA8 frame per view, clearing to
/// `clear_color` (linear RGBA in `[0, 1]`, matching `wgpu::Color`).
pub fn render_all_views_with(
    scene: &SceneData,
    views: &[ViewSpec],
    width: u32,
    height: u32,
    clear_color: [f64; 4],
) -> Result<Vec<Vec<u8>>, RenderError> {
    if scene.vertices.is_empty() || scene.indices.is_empty() {
        return Err(RenderError::EmptyScene);
    }
    if width == 0 || height == 0 {
        return Err(RenderError::InvalidSize { width, height });
    }
    validate_scene(scene)?;

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::LowPower,
        force_fallback_adapter: false,
        compatible_surface: None,
        apply_limit_buckets: false,
    }))
    .map_err(|_| RenderError::NoAdapter)?;

    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("avm-offscreen-device"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::downlevel_defaults(),
        experimental_features: wgpu::ExperimentalFeatures::disabled(),
        memory_hints: wgpu::MemoryHints::default(),
        trace: wgpu::Trace::Off,
    }))
    .map_err(|error| RenderError::Device(error.to_string()))?;

    let renderer = GpuSceneRenderer::new(&device, &queue, scene, COLOR_FORMAT)?;
    let color_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("avm-offscreen-color"),
        size: wgpu::Extent3d {
            width,
            height,
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
    let depth_view = create_depth_view(&device, width, height);

    let mut frames = Vec::with_capacity(views.len());
    for view in views {
        let readback = ReadbackTarget::new(&device, width, height);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("avm-offscreen-encoder"),
        });
        renderer.render(
            &queue,
            &mut encoder,
            &color_view,
            &depth_view,
            view,
            wgpu::Color {
                r: clear_color[0],
                g: clear_color[1],
                b: clear_color[2],
                a: clear_color[3],
            },
        );
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &color_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &readback.buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(readback.padded_bytes_per_row),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        queue.submit([encoder.finish()]);
        frames.push(readback.read(&device, width, height)?);
    }

    Ok(frames)
}

struct ReadbackTarget {
    buffer: wgpu::Buffer,
    padded_bytes_per_row: u32,
}

impl ReadbackTarget {
    const ALIGNMENT: u32 = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;

    fn new(device: &wgpu::Device, width: u32, height: u32) -> Self {
        let unpadded = width * 4;
        let padded_bytes_per_row = unpadded.div_ceil(Self::ALIGNMENT) * Self::ALIGNMENT;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("avm-offscreen-readback"),
            size: u64::from(padded_bytes_per_row) * u64::from(height),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        Self {
            buffer,
            padded_bytes_per_row,
        }
    }

    fn read(&self, device: &wgpu::Device, width: u32, height: u32) -> Result<Vec<u8>, RenderError> {
        let (sender, receiver) = std::sync::mpsc::channel();
        self.buffer
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = sender.send(result);
            });
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|error| RenderError::Readback(error.to_string()))?;
        receiver
            .recv()
            .map_err(|error| RenderError::Readback(error.to_string()))?
            .map_err(|error| RenderError::Readback(error.to_string()))?;

        let mut frame = Vec::with_capacity((width * height * 4) as usize);
        {
            let mapped = self
                .buffer
                .slice(..)
                .get_mapped_range()
                .map_err(|error| RenderError::Readback(error.to_string()))?;
            let row_bytes = (width * 4) as usize;
            for row in 0..height as usize {
                let start = row * self.padded_bytes_per_row as usize;
                frame.extend_from_slice(&mapped[start..start + row_bytes]);
            }
        }
        self.buffer.unmap();
        Ok(frame)
    }
}
