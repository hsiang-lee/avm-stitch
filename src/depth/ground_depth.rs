use crate::angle_utils::DEG_TO_RAD;
use crate::camera::CameraExtrinsics;
use crate::Vector3;

use super::rectification::{
    camera_direction, rectified_direction, CameraSide, RectificationBasis, SphericalGrid,
};

#[derive(Clone, Copy, Debug)]
pub struct GroundDepthOptions {
    pub ground_z: f64,
    pub min_depth: f64,
    pub max_depth: f64,
}

impl Default for GroundDepthOptions {
    fn default() -> Self {
        Self {
            ground_z: 0.0,
            min_depth: 0.5,
            max_depth: 100.0,
        }
    }
}

/// Dense row-major maps over the spherical grid; invalid cells stay zero.
pub struct GroundDepthMap {
    depth: Vec<f32>,
    disparity: Vec<f32>,
    valid: Vec<u8>,
    width: usize,
    height: usize,
}

impl GroundDepthMap {
    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn depth(&self, row: usize, col: usize) -> f32 {
        self.depth[row * self.width + col]
    }

    pub fn disparity(&self, row: usize, col: usize) -> f32 {
        self.disparity[row * self.width + col]
    }

    pub fn valid(&self, row: usize, col: usize) -> bool {
        self.valid[row * self.width + col] != 0
    }
}

struct GroundSample {
    depth: f32,
    disparity: f32,
}

struct GroundSampler<'a> {
    basis: &'a RectificationBasis,
    a: &'a CameraExtrinsics,
    b: &'a CameraExtrinsics,
    grid: &'a SphericalGrid,
    options: &'a GroundDepthOptions,
    psi_min: f64,
    psi_max: f64,
}

impl GroundSampler<'_> {
    fn sample(&self, row: usize, col: usize) -> Option<GroundSample> {
        let dir_r = self.grid.direction_c(row, col);
        let dir_a = camera_direction(self.basis, dir_r, CameraSide::First);
        let dir_world = self.a.rotate_camera_to_world(dir_a);

        let denom = dir_world[2];
        if denom.abs() < 1e-12 {
            return None;
        }
        let lambda = (self.options.ground_z - self.a.position()[2]) / denom;
        if !lambda.is_finite()
            || lambda < self.options.min_depth
            || lambda > self.options.max_depth
        {
            return None;
        }
        let point = self.a.position() + lambda * dir_world;

        let dir_b_world = (point - self.b.position()).normalize();
        let dir_rb = rectified_direction(self.basis, dir_b_world, self.b, CameraSide::Second);

        let psi_b = dir_rb[1].hypot(dir_rb[2]).atan2(dir_rb[0]);
        if psi_b < self.psi_min || psi_b > self.psi_max {
            return None;
        }
        let col_b = (psi_b - self.psi_min) / (self.psi_max - self.psi_min)
            * (self.grid.width - 1) as f64;

        Some(GroundSample {
            depth: lambda as f32,
            disparity: (col_b - col as f64) as f32,
        })
    }
}

pub fn compute_ground_depth_map(
    basis: &RectificationBasis,
    a: &CameraExtrinsics,
    b: &CameraExtrinsics,
    grid: &SphericalGrid,
    options: GroundDepthOptions,
) -> GroundDepthMap {
    let cells = grid.width * grid.height;
    let mut map = GroundDepthMap {
        depth: vec![0.0; cells],
        disparity: vec![0.0; cells],
        valid: vec![0; cells],
        width: grid.width,
        height: grid.height,
    };

    let psi_min = grid.psi_min_deg * DEG_TO_RAD;
    let psi_max = grid.psi_max_deg * DEG_TO_RAD;
    if grid.width < 2 || grid.height < 1 || psi_max <= psi_min {
        return map;
    }

    let sampler = GroundSampler {
        basis,
        a,
        b,
        grid,
        options: &options,
        psi_min,
        psi_max,
    };
    for row in 0..grid.height {
        for col in 0..grid.width {
            let Some(sample) = sampler.sample(row, col) else {
                continue;
            };
            let index = row * grid.width + col;
            map.depth[index] = sample.depth;
            map.disparity[index] = sample.disparity;
            map.valid[index] = 1;
        }
    }
    map
}

pub fn triangulate_depth(
    dir_a_world: Vector3<f64>,
    origin_a: Vector3<f64>,
    dir_b_world: Vector3<f64>,
    origin_b: Vector3<f64>,
) -> f64 {
    let aa = dir_a_world.dot(&dir_a_world);
    let bb = dir_b_world.dot(&dir_b_world);
    let ab = dir_a_world.dot(&dir_b_world);
    let det = aa * bb - ab * ab;
    if det.abs() < 1e-12 {
        return 0.0;
    }
    let c = origin_b - origin_a;
    let r0 = dir_a_world.dot(&c);
    let r1 = -dir_b_world.dot(&c);
    (bb * r0 + ab * r1) / det
}
