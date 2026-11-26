//! Bowl geometry parameters and block-boat mesh generation.

/// Outer radius of the calibrated reference bowl.
pub const CALIBRATED_ANCHOR_OUTER_RADIUS: f64 = 12.6;

#[derive(Clone, Debug, PartialEq)]
pub struct ViewPose {
    pub name: String,
    pub position: [f64; 3],
    pub target: [f64; 3],
    pub up: [f64; 3],
    pub fovy_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClipPlanes {
    pub near: f64,
    pub far: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub triangles: Vec<[usize; 3]>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BowlParams {
    car_length: f64,
    car_width: f64,
    flat_platform_radius: f64,
    grid_step: f64,
    outer_radius: f64,
    slope_coefficient: f64,
    height_exponent: f64,
    max_height: f64,
    car_center: [f64; 2],
}

impl BowlParams {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        car_length: f64,
        car_width: f64,
        flat_platform_radius: f64,
        grid_step: f64,
        outer_radius: f64,
        slope_coefficient: f64,
        height_exponent: f64,
        max_height: f64,
        car_center: [f64; 2],
    ) -> Self {
        Self {
            car_length,
            car_width,
            flat_platform_radius,
            grid_step,
            outer_radius,
            slope_coefficient,
            height_exponent,
            max_height,
            car_center,
        }
    }

    pub fn for_car_size(car_length: f64, car_width: f64, axis_x: f64) -> Self {
        const REFERENCE_CAR_LENGTH: f64 = 5.6;
        let scale = car_length / REFERENCE_CAR_LENGTH;

        Self::new(
            car_length,
            car_width,
            2.0 * scale,
            0.2 * scale,
            CALIBRATED_ANCHOR_OUTER_RADIUS * scale,
            0.09 * scale.powf(-1.2),
            2.2,
            8.6 * scale,
            [axis_x / 2.0, 0.0],
        )
    }

    pub fn grid_step(&self) -> f64 {
        self.grid_step
    }

    pub fn outer_radius(&self) -> f64 {
        self.outer_radius
    }

    pub fn flat_platform_radius(&self) -> f64 {
        self.flat_platform_radius
    }

    pub fn slope_coefficient(&self) -> f64 {
        self.slope_coefficient
    }

    pub fn height_exponent(&self) -> f64 {
        self.height_exponent
    }

    pub fn max_height(&self) -> f64 {
        self.max_height
    }

    pub fn car_center(&self) -> [f64; 2] {
        self.car_center
    }

    /// Distance from the flat platform rectangle (0 inside it).
    pub fn distance_from_flat_platform(&self, x: f64, y: f64) -> f64 {
        let half_car_length = self.car_length / 2.0;
        let half_car_width = self.car_width / 2.0;
        let offset_x = x - self.car_center[0];
        let offset_y = y - self.car_center[1];

        let overshoot_x =
            (offset_x.abs() - (half_car_length + self.flat_platform_radius)).max(0.0);
        let overshoot_y = (offset_y.abs() - (half_car_width + self.flat_platform_radius)).max(0.0);
        (overshoot_x * overshoot_x + overshoot_y * overshoot_y).sqrt()
    }

    pub fn bowl_height(&self, distance_from_flat_platform: f64) -> f64 {
        if distance_from_flat_platform <= 0.0 {
            return 0.0;
        }
        self.slope_coefficient * distance_from_flat_platform.powf(self.height_exponent)
    }

    pub fn grid_segments_per_axis(&self) -> i32 {
        // `0/0` rounds to NaN and `x/0` to inf; the guard turns either into 0
        // segments, which keeps an infinite ratio from casting to i32::MAX and
        // requesting an enormous mesh. It does not bound a finite but huge
        // ratio (e.g. outer_radius / grid_step = 1e12): that still reaches the
        // cast below and saturates to i32::MAX with the same huge allocation.
        let ratio = (self.outer_radius / self.grid_step).round();
        if !ratio.is_finite() {
            return 0;
        }
        (ratio * 2.0) as i32
    }

    pub fn generate_block_boat_mesh(&self) -> Mesh {
        let segments = self.grid_segments_per_axis();
        if segments < 0 {
            return Mesh::default();
        }
        let rows = segments as usize;
        let cols = rows;

        let mut mesh = Mesh::default();
        let mut grid_to_index = vec![-1i32; (rows + 1) * (cols + 1)];
        let mut vertex_count: i32 = 0;

        for i in 0..=rows {
            for j in 0..=cols {
                let u = i as f64 / rows as f64 * 2.0 - 1.0;
                let v = j as f64 / cols as f64 * 2.0 - 1.0;
                let x = self.car_center[0] + u * self.outer_radius;
                let y = self.car_center[1] + v * self.outer_radius;

                let distance_from_platform = self.distance_from_flat_platform(x, y);
                if distance_from_platform > self.outer_radius + 1e-9 {
                    continue;
                }

                let z = self.bowl_height(distance_from_platform);
                if z > self.max_height {
                    continue;
                }

                mesh.vertices.push([x, y, z]);
                mesh.normals.push([0.0, 0.0, 1.0]);
                grid_to_index[i * (cols + 1) + j] = vertex_count;
                vertex_count += 1;
            }
        }

        compute_bowl_normals(&grid_to_index, rows, cols, &mesh.vertices, &mut mesh.normals);

        for i in 0..rows {
            for j in 0..cols {
                let p1 = grid_to_index[i * (cols + 1) + j];
                let p2 = grid_to_index[i * (cols + 1) + (j + 1)];
                let p3 = grid_to_index[(i + 1) * (cols + 1) + j];
                let p4 = grid_to_index[(i + 1) * (cols + 1) + (j + 1)];
                if p1 >= 0 && p2 >= 0 && p3 >= 0 && p4 >= 0 {
                    mesh.triangles.push([p1 as usize, p3 as usize, p2 as usize]);
                    mesh.triangles.push([p2 as usize, p3 as usize, p4 as usize]);
                }
            }
        }

        mesh
    }

    pub fn default_view_poses(&self) -> Vec<ViewPose> {
        let scale = self.outer_radius / CALIBRATED_ANCHOR_OUTER_RADIUS;
        let cx = self.car_center[0];
        let cy = self.car_center[1];
        let z_ground = 0.2 * scale;
        let pose = |name: &str, position: [f64; 3]| ViewPose {
            name: name.to_string(),
            position,
            target: [cx, cy, z_ground],
            up: [0.0, 0.0, 1.0],
            fovy_deg: 60.0,
        };
        vec![
            pose("3d_front", [cx + 5.8 * scale, cy, 2.0 * scale]),
            pose("3d_rear", [cx - 5.8 * scale, cy, 2.0 * scale]),
            pose("3d_left", [cx, cy + 5.8 * scale, 2.0 * scale]),
            pose("3d_right", [cx, cy - 5.8 * scale, 2.0 * scale]),
        ]
    }

    pub fn clip_planes_for(&self, max_eye_to_target: f64) -> ClipPlanes {
        const FAR_MARGIN: f64 = 5.3553572528096565;
        const NEAR_REL: f64 = 0.01;
        let far = FAR_MARGIN * (max_eye_to_target + self.outer_radius);
        // `clamp(v, lo, hi)` is `min(max(v, lo), hi)`. Spelled with plain
        // comparisons, so a NaN radius propagates (Rust's `f64::max`/`min`
        // would silently drop the NaN), and `lo > hi` cannot panic (Rust's
        // `f64::clamp` would). Degenerate rule for the all-zero default
        // (`far == 0`): `hi` wins, so `near == far == 0` and the planes stay
        // ordered. Hardened standard libraries abort on this input, so there is
        // no numeric reference result to copy.
        let low = self.outer_radius * NEAR_REL;
        let raised = if low < 1.0e-3 { 1.0e-3 } else { low };
        let near = if far / 1000.0 < raised {
            far / 1000.0
        } else {
            raised
        };
        ClipPlanes { near, far }
    }

    pub fn home_view_pose(&self) -> ViewPose {
        let cx = self.car_center[0];
        let cy = self.car_center[1];
        let scale = self.outer_radius / CALIBRATED_ANCHOR_OUTER_RADIUS;
        ViewPose {
            name: String::new(),
            position: [
                10.0 * scale + cx,
                -6.0 * scale,
                6.0 * scale - cy,
            ],
            target: [cx, 0.0, 0.5 * scale - cy],
            up: [0.0, 0.0, 1.0],
            fovy_deg: 60.0,
        }
    }

    pub fn home_view_distance(&self) -> f64 {
        12.0 * self.outer_radius / CALIBRATED_ANCHOR_OUTER_RADIUS
    }
}

/// Central-difference normals of the height field (flat areas point up).
fn compute_bowl_normals(
    grid_to_index: &[i32],
    rows: usize,
    cols: usize,
    vertices: &[[f64; 3]],
    normals: &mut [[f64; 3]],
) {
    for i in 0..=rows {
        for j in 0..=cols {
            let idx = grid_to_index[i * (cols + 1) + j];
            if idx < 0 {
                continue;
            }
            let idx = idx as usize;
            let z = vertices[idx][2];
            if z < 1e-9 {
                normals[idx] = [0.0, 0.0, 1.0];
                continue;
            }

            let neighbor_dz = |ni: isize, nj: isize| -> Option<f64> {
                if ni < 0 || ni > rows as isize || nj < 0 || nj > cols as isize {
                    return None;
                }
                let nidx = grid_to_index[ni as usize * (cols + 1) + nj as usize];
                if nidx < 0 {
                    return None;
                }
                Some(vertices[nidx as usize][2] - z)
            };

            let mut dz_dx = 0.0;
            let mut dz_dy = 0.0;
            let mut count_x = 0.0;
            let mut count_y = 0.0;
            if let Some(d) = neighbor_dz(i as isize, j as isize - 1) {
                dz_dx -= d;
                count_x += 1.0;
            }
            if let Some(d) = neighbor_dz(i as isize, j as isize + 1) {
                dz_dx += d;
                count_x += 1.0;
            }
            if let Some(d) = neighbor_dz(i as isize - 1, j as isize) {
                dz_dy -= d;
                count_y += 1.0;
            }
            if let Some(d) = neighbor_dz(i as isize + 1, j as isize) {
                dz_dy += d;
                count_y += 1.0;
            }
            if count_x > 0.0 {
                dz_dx /= count_x;
            }
            if count_y > 0.0 {
                dz_dy /= count_y;
            }

            let n = [-dz_dx, -dz_dy, 1.0];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            normals[idx] = if len > 1e-9 {
                [n[0] / len, n[1] / len, n[2] / len]
            } else {
                [0.0, 0.0, 1.0]
            };
        }
    }
}
