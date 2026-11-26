//! `StitchEngine` orchestration: validate the calibration, bake the bowl once
//! and render the four default views.

use avm_core::bowl::{BowlModel, BowlParams};
use avm_core::camera::Camera;
use avm_render::{render_all_views_with, RenderError, RenderSize, SceneData, ViewSpec};
use image::RgbaImage;
use thiserror::Error;

/// `BackgroundClearColor()` of the C++ renderer.
const CLEAR_COLOR: [f64; 4] = [30.0 / 255.0, 36.0 / 255.0, 50.0 / 255.0, 1.0];

#[derive(Debug, Error)]
pub enum EngineError {
    #[error("StitchEngine::Configure: N must be in [1, 8], got {0}")]
    CameraCount(usize),
    #[error("StitchEngine::Configure: camera {0} has no model (Camera.model is null)")]
    MissingCameraModel(usize),
    #[error(
        "StitchEngine::Configure: all cameras must share the same image size; camera 0 is \
         {first_width}x{first_height}, camera {index} is {width}x{height}"
    )]
    ImageSizeMismatch {
        index: usize,
        first_width: i32,
        first_height: i32,
        width: i32,
        height: i32,
    },
    #[error("StitchEngine::Configure: bowl_params.GridStep must be > 0, got {0}")]
    GridStep(f64),
    #[error("StitchEngine::Configure: bowl_params.OuterRadius must be > 0, got {0}")]
    OuterRadius(f64),
    #[error("StitchEngine::Configure: bowl_params.SlopeCoefficient must be >= 0, got {0}")]
    SlopeCoefficient(f64),
    #[error("StitchEngine::Configure: bowl_params.HeightExponent must be >= 0, got {0}")]
    HeightExponent(f64),
    #[error("StitchEngine::RenderFrame called before Configure()")]
    NotConfigured,
    #[error("StitchEngine::RenderFrame: expected {expected} images, got {got}")]
    ImageCount { expected: usize, got: usize },
    #[error(transparent)]
    Render(#[from] RenderError),
}

/// A default view: its name and the matrix pair rendered for it travel
/// together so the label list can never drift from the view list.
struct NamedView {
    name: String,
    spec: ViewSpec,
}

struct Configured {
    scene: SceneData,
    views: Vec<NamedView>,
    output: (u32, u32),
}

/// Validates in the C++ order: the first failing check wins, so the null-model
/// gate stays ahead of the image-size comparison.
fn validate(cameras: &[Camera], bowl: &BowlParams) -> Result<(), EngineError> {
    let count = cameras.len();
    if count == 0 || count > 8 {
        return Err(EngineError::CameraCount(count));
    }
    for (index, camera) in cameras.iter().enumerate() {
        if !camera.is_valid() {
            return Err(EngineError::MissingCameraModel(index));
        }
    }
    let (first_width, first_height) = (cameras[0].width(), cameras[0].height());
    for (index, camera) in cameras.iter().enumerate().skip(1) {
        if camera.width() != first_width || camera.height() != first_height {
            return Err(EngineError::ImageSizeMismatch {
                index,
                first_width,
                first_height,
                width: camera.width(),
                height: camera.height(),
            });
        }
    }
    if bowl.grid_step() <= 0.0 {
        return Err(EngineError::GridStep(bowl.grid_step()));
    }
    if bowl.outer_radius() <= 0.0 {
        return Err(EngineError::OuterRadius(bowl.outer_radius()));
    }
    if bowl.slope_coefficient() < 0.0 {
        return Err(EngineError::SlopeCoefficient(bowl.slope_coefficient()));
    }
    if bowl.height_exponent() < 0.0 {
        return Err(EngineError::HeightExponent(bowl.height_exponent()));
    }
    Ok(())
}

/// VCS -> y-up render basis, matching the C++ `VcsToOgl`.
fn to_render_basis(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[2] as f32, -p[1] as f32]
}

fn build_views(bowl: &BowlParams, width: u32, height: u32) -> Vec<NamedView> {
    let poses = bowl.default_view_poses();
    let max_eye = poses.iter().fold(0.0f64, |max, pose| {
        let d = [
            pose.position[0] - pose.target[0],
            pose.position[1] - pose.target[1],
            pose.position[2] - pose.target[2],
        ];
        max.max((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt())
    });
    let clip = bowl.clip_planes_for(max_eye);
    let aspect = width as f32 / height as f32;
    poses
        .iter()
        .map(|pose| NamedView {
            name: pose.name.clone(),
            spec: ViewSpec::look_at(
                to_render_basis(pose.position),
                to_render_basis(pose.target),
                to_render_basis(pose.up),
                pose.fovy_deg as f32,
                aspect,
                clip.near as f32,
                clip.far as f32,
            ),
        })
        .collect()
}

#[derive(Default)]
pub struct StitchEngine {
    configured: Option<Configured>,
}

impl StitchEngine {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn configure(
        &mut self,
        cameras: Vec<Camera>,
        bowl: &BowlParams,
        width: u32,
        height: u32,
    ) -> Result<(), EngineError> {
        validate(&cameras, bowl)?;

        let model = BowlModel::new(bowl, &cameras);
        let scene = SceneData::from_bowl_model(&model);
        let views = build_views(bowl, width, height);

        self.configured = Some(Configured {
            scene,
            views,
            output: (width, height),
        });
        Ok(())
    }

    pub fn render_frame(
        &self,
        images: &[RgbaImage],
    ) -> Result<Vec<(String, RgbaImage)>, EngineError> {
        let configured = self.configured.as_ref().ok_or(EngineError::NotConfigured)?;
        if images.len() != configured.scene.num_cameras {
            return Err(EngineError::ImageCount {
                expected: configured.scene.num_cameras,
                got: images.len(),
            });
        }

        let mut scene = configured.scene.clone();
        scene.camera_images = images.to_vec();
        let (width, height) = configured.output;
        let specs: Vec<ViewSpec> = configured.views.iter().map(|view| view.spec).collect();
        let frames = render_all_views_with(&scene, &specs, RenderSize::new(width, height), CLEAR_COLOR)?;

        Ok(configured
            .views
            .iter()
            .zip(frames)
            .map(|(view, frame)| {
                let image = RgbaImage::from_raw(width, height, frame)
                    .expect("the renderer returns width * height RGBA pixels");
                (view.name.clone(), image)
            })
            .collect())
    }

    pub fn is_configured(&self) -> bool {
        self.configured.is_some()
    }

    pub fn num_cameras(&self) -> usize {
        self.configured
            .as_ref()
            .map_or(0, |configured| configured.scene.num_cameras)
    }
}
