//! Baked bowl model: mesh plus per-vertex uv/weight for every camera.

use nalgebra::Vector3;

use crate::bowl::blend::BlendWeightsCalculator;
use crate::bowl::params::BowlParams;
use crate::camera::{Camera, DEFAULT_EDGE_FEATHER_PX};

#[derive(Clone, Debug, PartialEq)]
pub struct BakedVertex {
    pub uv: Vec<[f32; 2]>,
    pub weight: Vec<f32>,
}

impl BakedVertex {
    pub fn new(num_cameras: usize) -> Self {
        Self {
            uv: vec![[0.0, 0.0]; num_cameras],
            weight: vec![0.0; num_cameras],
        }
    }
}

#[derive(Clone, Debug)]
pub struct BowlModel {
    vertices: Vec<[f64; 3]>,
    normals: Vec<[f64; 3]>,
    triangles: Vec<[usize; 3]>,
    baked_vertices: Vec<BakedVertex>,
}

impl BowlModel {
    pub fn new(params: &BowlParams, cameras: &[Camera]) -> Self {
        let mesh = params.generate_block_boat_mesh();
        let baked_vertices = bake_vertices(&mesh.vertices, cameras, params.car_center());
        Self {
            vertices: mesh.vertices,
            normals: mesh.normals,
            triangles: mesh.triangles,
            baked_vertices,
        }
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn vertex(&self, i: usize) -> &[f64; 3] {
        &self.vertices[i]
    }

    pub fn normal_count(&self) -> usize {
        self.normals.len()
    }

    pub fn normal(&self, i: usize) -> &[f64; 3] {
        &self.normals[i]
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn triangle(&self, i: usize) -> [usize; 3] {
        self.triangles[i]
    }

    pub fn baked_num_cameras(&self) -> usize {
        self.baked_vertices
            .first()
            .map(|v| v.weight.len())
            .unwrap_or(0)
    }

    pub fn baked_vertex(&self, i: usize) -> &BakedVertex {
        &self.baked_vertices[i]
    }

    pub fn has_visible_vertices(&self) -> bool {
        let num_cameras = self.baked_num_cameras();
        self.baked_vertices.iter().any(|baked_vertex| {
            let weight_sum: f64 = baked_vertex.weight[..num_cameras]
                .iter()
                .copied()
                .map(f64::from)
                .sum();
            weight_sum > 1e-9
        })
    }
}

/// Project every mesh vertex into every camera and normalize the edge-gated
/// blend weights (D3 fallback included).
fn bake_vertices(
    vertices: &[[f64; 3]],
    cameras: &[Camera],
    car_center: [f64; 2],
) -> Vec<BakedVertex> {
    let num_cameras = cameras.len();
    let mut baked_vertices = vec![BakedVertex::new(num_cameras); vertices.len()];
    let blend_calculator = BlendWeightsCalculator::new(cameras, car_center);
    let mut edge_factors = vec![0.0; num_cameras];

    for (vertex_index, world_position) in vertices.iter().enumerate() {
        let blend_weights = blend_calculator.weights_for(*world_position);
        let baked_vertex = &mut baked_vertices[vertex_index];

        for (camera_index, camera) in cameras.iter().enumerate() {
            let camera_point = camera.extrinsics().to_camera(Vector3::new(
                world_position[0],
                world_position[1],
                world_position[2],
            ));
            let (pixel_uv, inside_image_rect) = camera.project_point([
                camera_point.x,
                camera_point.y,
                camera_point.z,
            ]);

            let mut edge_factor = 0.0;
            if inside_image_rect != 0
                && camera.inside_fisheye_circle(pixel_uv[0], pixel_uv[1])
            {
                baked_vertex.uv[camera_index] = [
                    (pixel_uv[0] / f64::from(camera.width())) as f32,
                    (pixel_uv[1] / f64::from(camera.height())) as f32,
                ];
                edge_factor =
                    camera.edge_feather(pixel_uv[0], pixel_uv[1], DEFAULT_EDGE_FEATHER_PX);
            }
            edge_factors[camera_index] = edge_factor;
        }

        blend_calculator.normalize_weights(&blend_weights, &edge_factors, &mut baked_vertex.weight);
    }

    baked_vertices
}
