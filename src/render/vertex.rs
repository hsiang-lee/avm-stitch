//! Vertex packing. Attribute locations must match `shader.wgsl`:
//! 0 position vec3, 1 normal vec3, 2..5 uv slots vec4 (2 cameras each),
//! 6..7 weight slots vec4 (4 cameras each). Eight cameras fit in total.

use crate::bowl::{BakedVertex, BowlModel};

pub const NUM_UV_SLOTS: usize = 4;
pub const NUM_WEIGHT_SLOTS: usize = 2;
/// Cameras representable by the packed layout (`4 uv slots x 2`).
pub const MAX_CAMERAS: usize = NUM_UV_SLOTS * 2;

/// Size of [`PackedVertex`] in bytes (dense `f32` fields, no padding).
pub const PACKED_VERTEX_SIZE: usize = 120;

/// Positions and normals are stored in the y-up render basis:
/// `(x, y, z) -> (x, z, -y)`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PackedVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [[f32; 4]; NUM_UV_SLOTS],
    pub weights: [[f32; 4]; NUM_WEIGHT_SLOTS],
}

impl Default for PackedVertex {
    fn default() -> Self {
        Self {
            position: [0.0; 3],
            normal: [0.0; 3],
            uv: [[0.0; 4]; NUM_UV_SLOTS],
            weights: [[0.0; 4]; NUM_WEIGHT_SLOTS],
        }
    }
}

impl PackedVertex {
    /// Pack one baked vertex; missing cameras (and slots past 8) read as zero.
    pub fn from_baked(vertex: &[f64; 3], normal: &[f64; 3], baked: &BakedVertex) -> Self {
        let num_cameras = baked.weight.len();
        let mut packed = Self {
            position: [
                vertex[0] as f32,
                vertex[2] as f32,
                -vertex[1] as f32,
            ],
            normal: [normal[0] as f32, normal[2] as f32, -normal[1] as f32],
            ..Self::default()
        };

        for slot in 0..NUM_UV_SLOTS {
            let c0 = slot * 2;
            let c1 = c0 + 1;
            if c0 < num_cameras {
                packed.uv[slot][0] = baked.uv[c0][0];
                packed.uv[slot][1] = baked.uv[c0][1];
            }
            if c1 < num_cameras {
                packed.uv[slot][2] = baked.uv[c1][0];
                packed.uv[slot][3] = baked.uv[c1][1];
            }
        }

        for slot in 0..NUM_WEIGHT_SLOTS {
            for lane in 0..4 {
                let camera = slot * 4 + lane;
                if camera < num_cameras {
                    packed.weights[slot][lane] = baked.weight[camera];
                }
            }
        }

        packed
    }

    pub fn vertex_buffer(model: &BowlModel) -> Vec<PackedVertex> {
        (0..model.vertex_count())
            .map(|i| {
                PackedVertex::from_baked(model.vertex(i), model.normal(i), model.baked_vertex(i))
            })
            .collect()
    }

    pub fn index_buffer(model: &BowlModel) -> Vec<u32> {
        let mut indices = Vec::with_capacity(model.triangle_count() * 3);
        for t in 0..model.triangle_count() {
            let triangle = model.triangle(t);
            indices.push(triangle[0] as u32);
            indices.push(triangle[1] as u32);
            indices.push(triangle[2] as u32);
        }
        indices
    }

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: PACKED_VERTEX_SIZE as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: VertexAttributes::SLICE,
        }
    }
}

pub struct VertexAttributes;

impl VertexAttributes {
    pub const ATTRS: [wgpu::VertexAttribute; 8] = wgpu::vertex_attr_array![
        0 => Float32x3,
        1 => Float32x3,
        2 => Float32x4,
        3 => Float32x4,
        4 => Float32x4,
        5 => Float32x4,
        6 => Float32x4,
        7 => Float32x4,
    ];

    pub const SLICE: &'static [wgpu::VertexAttribute] = &Self::ATTRS;
}
