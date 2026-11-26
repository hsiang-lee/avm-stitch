//! Headless (offscreen) rendering: render target texture -> readback buffer.
//!
//! This is the skeleton the later batches build on. It keeps the C++ render
//! semantics (one blended pass over the baked bowl, camera array sampling with
//! the hard weight gate) and returns one RGBA8 frame per view.

use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;
use thiserror::Error;

use crate::scene::SceneData;
use crate::vertex::{PackedVertex, MAX_CAMERAS};
use crate::view::ViewSpec;
use crate::SHADER_SOURCE;

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
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Uniform block: view-projection matrix + active camera count. The 12-byte
/// pad after the `u32` rounds the 80-byte struct up to the 16-byte uniform
/// alignment required for bind group 0.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    num_cameras: u32,
    _pad: [u32; 3],
}

struct GpuContext {
    pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    camera_bind_group: wgpu::BindGroup,
}

/// Render every view offscreen and return one RGBA8 frame per view.
///
/// Returns [`RenderError::NoAdapter`] (never panics) when no GPU adapter is
/// available; callers/tests treat that as "skipped".
pub fn render_all_views(
    scene: &SceneData,
    views: &[ViewSpec],
    width: u32,
    height: u32,
) -> Result<Vec<Vec<u8>>, RenderError> {
    if scene.vertices.is_empty() || scene.indices.is_empty() {
        return Err(RenderError::EmptyScene);
    }
    if width == 0 || height == 0 {
        return Err(RenderError::InvalidSize { width, height });
    }
    // Mirrors the C++ `Configure` precondition (N9): the packed vertex layout
    // only carries 8 cameras, and every active camera needs a texture layer.
    if scene.num_cameras > MAX_CAMERAS {
        return Err(RenderError::TooManyCameras {
            num_cameras: scene.num_cameras,
            max: MAX_CAMERAS,
        });
    }
    if scene.camera_images.len() != scene.num_cameras {
        return Err(RenderError::CameraImageCountMismatch {
            num_cameras: scene.num_cameras,
            images: scene.camera_images.len(),
        });
    }

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

    let context = GpuContext::new(&device, &queue, scene)?;
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
    let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("avm-offscreen-depth"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("avm-offscreen-vertices"),
        contents: bytemuck::cast_slice(&scene.vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("avm-offscreen-indices"),
        contents: bytemuck::cast_slice(&scene.indices),
        usage: wgpu::BufferUsages::INDEX,
    });

    let mut frames = Vec::with_capacity(views.len());
    for view in views {
        let uniforms = Uniforms {
            view_proj: view.view_proj,
            num_cameras: scene.num_cameras as u32,
            _pad: [0; 3],
        };
        queue.write_buffer(&context.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));

        let readback = ReadbackTarget::new(&device, width, height);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("avm-offscreen-encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("avm-offscreen-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.0,
                            g: 0.0,
                            b: 0.0,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&context.pipeline);
            pass.set_bind_group(0, &context.uniform_bind_group, &[]);
            pass.set_bind_group(1, &context.camera_bind_group, &[]);
            pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..scene.indices.len() as u32, 0, 0..1);
        }
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

impl GpuContext {
    fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        scene: &SceneData,
    ) -> Result<Self, RenderError> {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("avm-offscreen-shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER_SOURCE.into()),
        });

        let uniform_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("avm-uniform-layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("avm-camera-layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2Array,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("avm-pipeline-layout"),
            bind_group_layouts: &[Some(&uniform_layout), Some(&camera_layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("avm-offscreen-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[Some(PackedVertex::layout())],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });

        // Camera layers: one texture array layer per camera. wgpu requires at
        // least one layer, so an empty camera list still gets a black layer
        // (the shader activates no layer because num_cameras is 0).
        let camera_size = scene
            .camera_images
            .first()
            .map(|image| (image.width(), image.height()))
            .unwrap_or((1, 1));
        for (index, image) in scene.camera_images.iter().enumerate() {
            if (image.width(), image.height()) != camera_size {
                return Err(RenderError::ImageSizeMismatch {
                    index,
                    width: image.width(),
                    height: image.height(),
                    expected_width: camera_size.0,
                    expected_height: camera_size.1,
                });
            }
        }
        let camera_layers = scene.camera_images.len().max(1) as u32;
        let camera_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("avm-camera-array"),
            size: wgpu::Extent3d {
                width: camera_size.0,
                height: camera_size.1,
                depth_or_array_layers: camera_layers,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let black = vec![0u8; (camera_size.0 * camera_size.1 * 4) as usize];
        for layer in 0..camera_layers {
            let data = scene
                .camera_images
                .get(layer as usize)
                .map(|image| image.as_raw().as_slice())
                .unwrap_or(black.as_slice());
            queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &camera_texture,
                    mip_level: 0,
                    origin: wgpu::Origin3d {
                        x: 0,
                        y: 0,
                        z: layer,
                    },
                    aspect: wgpu::TextureAspect::All,
                },
                data,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(camera_size.0 * 4),
                    rows_per_image: Some(camera_size.1),
                },
                wgpu::Extent3d {
                    width: camera_size.0,
                    height: camera_size.1,
                    depth_or_array_layers: 1,
                },
            );
        }
        let camera_view = camera_texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some("avm-camera-array-view"),
            format: None,
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            usage: None,
            aspect: wgpu::TextureAspect::All,
            base_mip_level: 0,
            mip_level_count: None,
            base_array_layer: 0,
            array_layer_count: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("avm-camera-sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Nearest,
            ..Default::default()
        });

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("avm-uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("avm-uniform-bind-group"),
            layout: &uniform_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("avm-camera-bind-group"),
            layout: &camera_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&camera_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
        });

        Ok(Self {
            pipeline,
            uniform_buffer,
            uniform_bind_group,
            camera_bind_group,
        })
    }
}

/// Staging buffer + row padding for the texture readback.
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
