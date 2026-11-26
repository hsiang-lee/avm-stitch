//! Trackball-style camera state. Pure math: no winit, no wgpu, no display.

use avm_core::bowl::{BowlParams, ClipPlanes, ViewPose};
use avm_render::ViewSpec;

/// Radians of orbit per mouse pixel.
const ROTATE_RAD_PER_PIXEL: f64 = 0.005;
/// Dolly factor per mouse pixel of vertical drag.
const ZOOM_PER_PIXEL: f64 = 0.01;
/// Keep the view direction off the poles so the up vector stays usable.
const MAX_PITCH_DEG: f64 = 89.0;

/// VCS (z up) -> render basis (y up), the same remap as the renderer.
fn render_basis(p: [f64; 3]) -> [f64; 3] {
    [p[0], p[2], -p[1]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(v: [f64; 3]) -> f64 {
    dot(v, v).sqrt()
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = norm(v);
    if n > 0.0 {
        scale(v, 1.0 / n)
    } else {
        v
    }
}

/// Which trackball operation a drag performs. The window maps mouse buttons to
/// this; the priority is left, then right, then middle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DragMode {
    Rotate,
    Zoom,
    Pan,
}

pub fn drag_mode(left: bool, right: bool, middle: bool) -> Option<DragMode> {
    if left {
        Some(DragMode::Rotate)
    } else if right {
        Some(DragMode::Zoom)
    } else if middle {
        Some(DragMode::Pan)
    } else {
        None
    }
}

#[derive(Clone, Debug)]
pub struct OrbitController {
    target: [f64; 3],
    distance: f64,
    yaw: f64,
    pitch: f64,
    up: [f64; 3],
    clip: ClipPlanes,
    fovy_deg: f64,
}

impl OrbitController {
    pub fn from_home(bowl: &BowlParams) -> Self {
        let pose = bowl.home_view_pose();
        let eye = render_basis(pose.position);
        let target = render_basis(pose.target);
        let clip = bowl.clip_planes_for(norm(sub(eye, target)));
        Self::from_pose(&pose, clip)
    }

    pub fn from_pose(pose: &ViewPose, clip: ClipPlanes) -> Self {
        let eye = render_basis(pose.position);
        let target = render_basis(pose.target);
        let offset = sub(eye, target);
        let distance = norm(offset);
        let (yaw, pitch) = if distance > 0.0 {
            (
                offset[0].atan2(offset[2]),
                (offset[1] / distance).clamp(-1.0, 1.0).asin(),
            )
        } else {
            (0.0, 0.0)
        };
        Self {
            target,
            distance,
            yaw,
            pitch,
            up: normalize(render_basis(pose.up)),
            clip,
            fovy_deg: pose.fovy_deg,
        }
    }

    fn direction(&self) -> [f64; 3] {
        let (sin_pitch, cos_pitch) = self.pitch.sin_cos();
        [
            cos_pitch * self.yaw.sin(),
            sin_pitch,
            cos_pitch * self.yaw.cos(),
        ]
    }

    pub fn eye(&self) -> [f64; 3] {
        add(self.target, scale(self.direction(), self.distance))
    }

    pub fn target(&self) -> [f64; 3] {
        self.target
    }

    pub fn up(&self) -> [f64; 3] {
        self.up
    }

    pub fn distance(&self) -> f64 {
        self.distance
    }

    pub fn clip(&self) -> ClipPlanes {
        self.clip
    }

    pub fn pitch_deg(&self) -> f64 {
        self.pitch.to_degrees()
    }

    pub fn rotate(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * ROTATE_RAD_PER_PIXEL;
        let limit = MAX_PITCH_DEG.to_radians();
        self.pitch = (self.pitch + dy * ROTATE_RAD_PER_PIXEL).clamp(-limit, limit);
    }

    /// Positive `dy` (drag up) dollies in. The result stays inside the fixed
    /// projection clip planes, since the window uses one projection matrix.
    pub fn zoom(&mut self, dy: f64) {
        let next = self.distance * (-dy * ZOOM_PER_PIXEL).exp();
        self.distance = if self.clip.far > self.clip.near && self.clip.near > 0.0 {
            next.clamp(self.clip.near, self.clip.far)
        } else {
            next
        };
    }

    /// Moves `target` in the camera plane; `eye` follows because it is derived
    /// from `target + distance * direction`.
    pub fn pan(&mut self, dx: f64, dy: f64, viewport_height: u32) {
        if viewport_height == 0 {
            return;
        }
        let forward = scale(self.direction(), -1.0);
        let right = normalize(cross(forward, self.up));
        let camera_up = cross(right, forward);
        let world_per_pixel = 2.0
            * self.distance
            * (0.5 * self.fovy_deg.to_radians()).tan()
            / f64::from(viewport_height);
        let shift = add(
            scale(right, -dx * world_per_pixel),
            scale(camera_up, dy * world_per_pixel),
        );
        self.target = add(self.target, shift);
    }

    /// Applies one drag step with the C++ control mapping: LMB rotates, RMB
    /// dollies and MMB pans.
    pub fn drag(&mut self, mode: DragMode, dx: f64, dy: f64, viewport_height: u32) {
        match mode {
            DragMode::Rotate => self.rotate(dx, dy),
            DragMode::Zoom => self.zoom(-dy),
            DragMode::Pan => self.pan(dx, dy, viewport_height),
        }
    }

    pub fn view_spec(&self, aspect: f32) -> ViewSpec {
        let eye = self.eye();
        let target = self.target;
        let up = self.up;
        ViewSpec::look_at(
            [eye[0] as f32, eye[1] as f32, eye[2] as f32],
            [target[0] as f32, target[1] as f32, target[2] as f32],
            [up[0] as f32, up[1] as f32, up[2] as f32],
            self.fovy_deg as f32,
            aspect,
            self.clip.near as f32,
            self.clip.far as f32,
        )
    }
}
