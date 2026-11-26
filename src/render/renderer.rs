//! Shared draw path for the offscreen renderer and the interactive window.

use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::render::offscreen::RenderError;
use crate::render::scene::SceneData;
use crate::render::vertex::{PackedVertex, MAX_CAMERAS};
use crate::render::view::ViewSpec;
use crate::render::SHADER_SOURCE;

pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// Camera images are uploaded as RGBA8, so this texture stays RGBA even when the
/// window surface prefers BGRA.
pub const CAMERA_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

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

/// Validates the scene properties both render paths depend on; the offscreen
/// path additionally rejects a zero-sized target.
pub fn validate_scene(scene: &SceneData) -> Result<(), RenderError> {
    if scene.vertices.is_empty() || scene.indices.is_empty() {
        return Err(RenderError::EmptyScene);
    }
    // The packed layout carries 8 cameras and every active camera needs a layer.
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
    Ok(())
}

/// Pixel dimensions of a render target; width and height always travel
/// together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderSize {
    pub width: u32,
    pub height: u32,
}

impl RenderSize {
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }
}

/// The color and depth attachments of one frame, paired so a caller cannot
/// transpose them positionally.
#[derive(Clone, Copy)]
pub struct RenderTarget<'a> {
    pub color: &'a wgpu::TextureView,
    pub depth: &'a wgpu::TextureView,
}

pub struct GpuSceneRenderer {
    pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    camera_bind_group: wgpu::BindGroup,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    num_cameras: u32,
}

impl GpuSceneRenderer {
    /// Builds the pipeline and the scene resources for `color_format`, which
    /// must match the render target (offscreen texture or window surface).
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        scene: &SceneData,
        color_format: wgpu::TextureFormat,
    ) -> Result<Self, RenderError> {
        validate_scene(scene)?;
        let (uniform_layout, camera_layout) = create_bind_group_layouts(device);
        let pipeline = create_pipeline(device, color_format, &uniform_layout, &camera_layout);
        let camera_bind_group = create_camera_bind_group(device, queue, scene, &camera_layout)?;
        let (uniform_buffer, uniform_bind_group) =
            create_uniform_resources(device, &uniform_layout);
        let (vertex_buffer, index_buffer) = create_scene_buffers(device, scene);

        Ok(Self {
            pipeline,
            uniform_buffer,
            uniform_bind_group,
            camera_bind_group,
            vertex_buffer,
            index_buffer,
            index_count: scene.indices.len() as u32,
            num_cameras: scene.num_cameras as u32,
        })
    }

    /// Records one full-screen pass for `view`; the caller owns submit and
    /// presentation.
    pub fn render(
        &self,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        target: RenderTarget<'_>,
        view: &ViewSpec,
        clear: wgpu::Color,
    ) {
        let uniforms = Uniforms {
            view_proj: view.view_proj,
            num_cameras: self.num_cameras,
            _pad: [0; 3],
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));

        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("avm-scene-pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target.color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: target.depth,
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
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.uniform_bind_group, &[]);
        pass.set_bind_group(1, &self.camera_bind_group, &[]);
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}

fn create_bind_group_layouts(
    device: &wgpu::Device,
) -> (wgpu::BindGroupLayout, wgpu::BindGroupLayout) {
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
    (uniform_layout, camera_layout)
}

fn create_pipeline(
    device: &wgpu::Device,
    color_format: wgpu::TextureFormat,
    uniform_layout: &wgpu::BindGroupLayout,
    camera_layout: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("avm-scene-shader"),
        source: wgpu::ShaderSource::Wgsl(SHADER_SOURCE.into()),
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("avm-pipeline-layout"),
        bind_group_layouts: &[Some(uniform_layout), Some(camera_layout)],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("avm-scene-pipeline"),
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
                format: color_format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// Texture array with one layer per camera plus a sampler, bound as group 1.
fn create_camera_bind_group(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    scene: &SceneData,
    layout: &wgpu::BindGroupLayout,
) -> Result<wgpu::BindGroup, RenderError> {
    let camera_view = upload_camera_array(device, queue, scene)?;
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
    Ok(device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("avm-camera-bind-group"),
        layout,
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
    }))
}

/// Validates the camera image sizes, then uploads one layer per camera.
/// wgpu requires at least one layer, so an empty camera list still gets a
/// black layer (the shader activates no layer because num_cameras is 0).
fn upload_camera_array(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    scene: &SceneData,
) -> Result<wgpu::TextureView, RenderError> {
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
        format: CAMERA_FORMAT,
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
    Ok(camera_texture.create_view(&wgpu::TextureViewDescriptor {
        label: Some("avm-camera-array-view"),
        format: None,
        dimension: Some(wgpu::TextureViewDimension::D2Array),
        usage: None,
        aspect: wgpu::TextureAspect::All,
        base_mip_level: 0,
        mip_level_count: None,
        base_array_layer: 0,
        array_layer_count: None,
    }))
}

/// Uniform buffer holding the per-view matrix, bound as group 0.
fn create_uniform_resources(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
) -> (wgpu::Buffer, wgpu::BindGroup) {
    let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("avm-uniforms"),
        size: std::mem::size_of::<Uniforms>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("avm-uniform-bind-group"),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: uniform_buffer.as_entire_binding(),
        }],
    });
    (uniform_buffer, uniform_bind_group)
}

fn create_scene_buffers(device: &wgpu::Device, scene: &SceneData) -> (wgpu::Buffer, wgpu::Buffer) {
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("avm-scene-vertices"),
        contents: bytemuck::cast_slice(&scene.vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("avm-scene-indices"),
        contents: bytemuck::cast_slice(&scene.indices),
        usage: wgpu::BufferUsages::INDEX,
    });
    (vertex_buffer, index_buffer)
}

pub fn create_depth_view(device: &wgpu::Device, size: RenderSize) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("avm-depth"),
            size: wgpu::Extent3d {
                width: size.width,
                height: size.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}
