//! Sector-based blend weights around the car centre.
//!
//! Port of `src/bowl_model/blend_weights.{hpp,cpp}`.

use crate::angle_utils::{normalize_angle_deg, RAD_TO_DEG, PI};
use crate::camera::Camera;

fn shortest_arc_deg(angle_a_deg: f64, angle_b_deg: f64) -> f64 {
    let distance_deg = normalize_angle_deg(angle_a_deg - angle_b_deg).abs();
    if distance_deg > 180.0 {
        360.0 - distance_deg
    } else {
        distance_deg
    }
}

fn sector_boundary_deg(azimuth_a_deg: f64, azimuth_b_deg: f64) -> f64 {
    let mut b = azimuth_b_deg;
    if b < azimuth_a_deg {
        b += 360.0;
    }
    normalize_angle_deg((azimuth_a_deg + b) / 2.0)
}

#[derive(Clone, Debug, PartialEq)]
pub struct BlendWeightsCalculator {
    cameras_by_azimuth: Vec<(f64, usize)>,
    pole: [f64; 2],
    num_cameras: usize,
}

impl BlendWeightsCalculator {
    /// Cameras are ordered by azimuth around `pole`.
    pub fn new(cameras: &[Camera], pole: [f64; 2]) -> Self {
        let mut cameras_by_azimuth: Vec<(f64, usize)> = cameras
            .iter()
            .enumerate()
            .map(|(i, cam)| (cam.extrinsics().azimuth_deg(), i))
            .collect();
        cameras_by_azimuth.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.1.cmp(&b.1))
        });
        Self {
            cameras_by_azimuth,
            pole,
            num_cameras: cameras.len(),
        }
    }

    pub fn with_default_pole(cameras: &[Camera]) -> Self {
        Self::new(cameras, [0.0, 0.0])
    }

    /// Per-original-index blend weights for a world position.
    pub fn weights_for(&self, world_position: [f64; 3]) -> Vec<f64> {
        if self.num_cameras == 0 {
            return Vec::new();
        }
        let mut weights = vec![0.0; self.num_cameras];

        let azimuth_deg = normalize_angle_deg(
            (world_position[1] - self.pole[1]).atan2(world_position[0] - self.pole[0]) * RAD_TO_DEG,
        );
        if self.num_cameras == 1 {
            weights[self.cameras_by_azimuth[0].1] = 1.0;
            return weights;
        }

        let blend_half_deg = Self::compute_blend_half_angle(self.num_cameras as i32);
        let nearest_index = self.nearest_camera_index(azimuth_deg);

        if self.try_blend(
            (nearest_index + self.num_cameras - 1) % self.num_cameras,
            azimuth_deg,
            blend_half_deg,
            nearest_index,
            &mut weights,
        ) {
            return weights;
        }
        if self.try_blend(
            nearest_index,
            azimuth_deg,
            blend_half_deg,
            nearest_index,
            &mut weights,
        ) {
            return weights;
        }

        weights[self.cameras_by_azimuth[nearest_index].1] = 1.0;
        weights
    }

    /// Edge-factor weighted normalization; entries are written for cameras that
    /// contributed, others stay untouched.
    pub fn normalize_weights(&self, blend_weights: &[f64], edge_factors: &[f64], out: &mut [f32]) {
        assert_eq!(blend_weights.len(), self.num_cameras);
        assert_eq!(edge_factors.len(), self.num_cameras);
        assert_eq!(out.len(), self.num_cameras);

        let mut visible_contrib_sum = 0.0;
        let mut edge_only_sum = 0.0;
        for index in 0..self.num_cameras {
            if edge_factors[index] > 0.0 {
                if blend_weights[index] > 1e-9 {
                    visible_contrib_sum += blend_weights[index] * edge_factors[index];
                }
                edge_only_sum += edge_factors[index];
            }
        }

        if visible_contrib_sum > 1e-9 {
            for index in 0..self.num_cameras {
                if edge_factors[index] > 0.0 && blend_weights[index] > 1e-9 {
                    out[index] = (blend_weights[index] * edge_factors[index]
                        / visible_contrib_sum) as f32;
                }
            }
        } else if edge_only_sum > 1e-9 {
            for index in 0..self.num_cameras {
                if edge_factors[index] > 0.0 {
                    out[index] = (edge_factors[index] / edge_only_sum) as f32;
                }
            }
        }
    }

    /// Half width of the blending band in degrees (`52 / N`).
    pub fn compute_blend_half_angle(num_cameras: i32) -> f64 {
        if num_cameras <= 0 {
            return 0.0;
        }
        52.0 / f64::from(num_cameras)
    }

    fn nearest_camera_index(&self, theta_deg: f64) -> usize {
        let mut nearest_index = 0;
        let mut best_dist = 360.0;
        for (camera_index, (azimuth, _)) in self.cameras_by_azimuth.iter().enumerate() {
            let distance_deg = shortest_arc_deg(theta_deg, *azimuth);
            if distance_deg < best_dist {
                best_dist = distance_deg;
                nearest_index = camera_index;
            }
        }
        nearest_index
    }

    fn try_blend(
        &self,
        sector_start_index: usize,
        theta_deg: f64,
        blend_half_deg: f64,
        nearest_index: usize,
        weights: &mut [f64],
    ) -> bool {
        let next_index = (sector_start_index + 1) % self.num_cameras;
        let boundary = sector_boundary_deg(
            self.cameras_by_azimuth[sector_start_index].0,
            self.cameras_by_azimuth[next_index].0,
        );
        let dist = shortest_arc_deg(theta_deg, boundary);
        if dist > blend_half_deg {
            return false;
        }
        let blend_ratio = 0.5 + dist / (2.0 * blend_half_deg);
        let weight_near = 0.5 * (1.0 - (blend_ratio * PI).cos());
        let weight_far = 1.0 - weight_near;
        if nearest_index == sector_start_index {
            weights[self.cameras_by_azimuth[sector_start_index].1] = weight_near;
            weights[self.cameras_by_azimuth[next_index].1] = weight_far;
        } else {
            weights[self.cameras_by_azimuth[next_index].1] = weight_near;
            weights[self.cameras_by_azimuth[sector_start_index].1] = weight_far;
        }
        true
    }
}
