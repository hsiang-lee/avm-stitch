//! Configure-time validation runs everywhere; the render cases skip when the
//! machine has no adapter.

use std::sync::Mutex;

use avm_core::bowl::BowlParams;
use avm_core::camera::{
    make_camera_model, Camera, CameraExtrinsics, CameraModelParams, KbParams,
};
use avm_engine::{EngineError, StitchEngine};
use image::{Rgba, RgbaImage};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 640;
const CAR_LENGTH: f64 = 5.6;
const CAR_WIDTH: f64 = 2.2;
const CAR_AXIS_X: f64 = 2.9;

static GPU_LOCK: Mutex<()> = Mutex::new(());

fn gpu_lock() -> std::sync::MutexGuard<'static, ()> {
    GPU_LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn kb_params() -> KbParams {
    KbParams {
        width: 640,
        height: 480,
        fx: 300.0,
        fy: 300.0,
        cx: 320.0,
        cy: 240.0,
        k1: 0.0,
        k2: 0.0,
        k3: 0.0,
        k4: 0.0,
        fisheye_radius: 0.0,
    }
}

fn make_car_bowl() -> BowlParams {
    BowlParams::for_car_size(CAR_LENGTH, CAR_WIDTH, CAR_AXIS_X)
}

/// The car-sized bowl with one or two fields overridden; the other fields keep
/// their derived values so a case states only what it is probing.
struct BowlVariant {
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

impl BowlVariant {
    fn from_car_bowl() -> Self {
        let derived = make_car_bowl();
        Self {
            car_length: CAR_LENGTH,
            car_width: CAR_WIDTH,
            flat_platform_radius: derived.flat_platform_radius(),
            grid_step: derived.grid_step(),
            outer_radius: derived.outer_radius(),
            slope_coefficient: derived.slope_coefficient(),
            height_exponent: derived.height_exponent(),
            max_height: derived.max_height(),
            car_center: derived.car_center(),
        }
    }

    fn car_size(mut self, car_length: f64, car_width: f64) -> Self {
        self.car_length = car_length;
        self.car_width = car_width;
        self
    }

    fn grid_step(mut self, grid_step: f64) -> Self {
        self.grid_step = grid_step;
        self
    }

    fn outer_radius(mut self, outer_radius: f64) -> Self {
        self.outer_radius = outer_radius;
        self
    }

    fn slope_coefficient(mut self, slope_coefficient: f64) -> Self {
        self.slope_coefficient = slope_coefficient;
        self
    }

    fn height_exponent(mut self, height_exponent: f64) -> Self {
        self.height_exponent = height_exponent;
        self
    }

    fn car_center(mut self, car_center: [f64; 2]) -> Self {
        self.car_center = car_center;
        self
    }

    fn build(self) -> BowlParams {
        BowlParams::new(
            self.car_length,
            self.car_width,
            self.flat_platform_radius,
            self.grid_step,
            self.outer_radius,
            self.slope_coefficient,
            self.height_exponent,
            self.max_height,
            self.car_center,
        )
    }
}

fn wrap_kb_cameras(intrinsics: &[KbParams], extrinsics: &[CameraExtrinsics]) -> Vec<Camera> {
    intrinsics
        .iter()
        .zip(extrinsics)
        .map(|(intr, ext)| {
            let model = make_camera_model("kb", &CameraModelParams::Kb(intr.clone()))
                .expect("kb fixture must build");
            Camera::new(*ext, model)
        })
        .collect()
}

fn make_look_at_camera(az_deg: f64, r: f64, h: f64, target_z: f64) -> CameraExtrinsics {
    let rad = az_deg.to_radians();
    let position = [r * rad.cos(), r * rad.sin(), h];
    let target = [0.0, 0.0, target_z];
    let forward = normalize([
        target[0] - position[0],
        target[1] - position[1],
        target[2] - position[2],
    ]);
    let up = [0.0, 0.0, 1.0];
    let right = normalize(cross(forward, up));
    let u = cross(right, forward);
    let down = [-u[0], -u[1], -u[2]];
    CameraExtrinsics::from_rows(
        [
            [right[0], down[0], forward[0]],
            [right[1], down[1], forward[1]],
            [right[2], down[2], forward[2]],
        ],
        position,
    )
}

fn make_look_at_cameras(num_cameras: usize) -> Vec<CameraExtrinsics> {
    (0..num_cameras)
        .map(|i| make_look_at_camera(i as f64 * 360.0 / num_cameras as f64, 4.0, 2.0, 0.0))
        .collect()
}

fn wrap_kb_side_cameras(azimuths_deg: &[f64]) -> Vec<Camera> {
    let extrinsics: Vec<CameraExtrinsics> = azimuths_deg
        .iter()
        .map(|az| {
            let rad = az.to_radians();
            let position = [3.0 * rad.cos(), 3.0 * rad.sin(), 1.0];
            let forward = [rad.cos(), rad.sin(), 0.0];
            let down = [0.0, 0.0, -1.0];
            let right = cross(down, forward);
            CameraExtrinsics::from_rows(
                [
                    [right[0], down[0], forward[0]],
                    [right[1], down[1], forward[1]],
                    [right[2], down[2], forward[2]],
                ],
                position,
            )
        })
        .collect();
    wrap_kb_cameras(&vec![kb_params(); azimuths_deg.len()], &extrinsics)
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / norm, v[1] / norm, v[2] / norm]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn colour_frames(num_cameras: usize) -> Vec<RgbaImage> {
    const COLOURS: [[u8; 3]; 4] = [
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 0],
    ];
    (0..num_cameras)
        .map(|i| {
            let colour = COLOURS[i % 4];
            RgbaImage::from_pixel(640, 480, Rgba([colour[0], colour[1], colour[2], 255]))
        })
        .collect()
}

fn count_vivid_pixels(image: &RgbaImage) -> usize {
    image
        .pixels()
        .filter(|p| {
            let (min, max) = (
                p[0].min(p[1]).min(p[2]),
                p[0].max(p[1]).max(p[2]),
            );
            max - min > 100
        })
        .count()
}

/// FNV-1a over the RGBA bytes: stable across runs, good enough to tell two
/// rendered frames apart.
fn content_hash(image: &RgbaImage) -> u64 {
    image.as_raw().iter().fold(0xcbf2_9ce4_8422_2325u64, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn band_frame() -> RgbaImage {
    const BANDS: [[u8; 3]; 6] = [
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 0],
        [255, 0, 255],
        [0, 255, 255],
    ];
    RgbaImage::from_fn(640, 480, |_x, y| {
        let band = BANDS[(y / 80) as usize];
        Rgba([band[0], band[1], band[2], 255])
    })
}

fn match_band(px: &Rgba<u8>) -> Option<usize> {
    const BANDS: [[u8; 3]; 6] = [
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 0],
        [255, 0, 255],
        [0, 255, 255],
    ];
    BANDS.iter().position(|b| b == &[px[0], px[1], px[2]])
}

fn dominant_band(image: &RgbaImage, y0: u32, y1: u32) -> Option<usize> {
    let mut counts = [0usize; 6];
    let x0 = image.width() / 2 - 30;
    let x1 = image.width() / 2 + 30;
    for y in y0..y1.min(image.height()) {
        for x in x0..x1 {
            if let Some(band) = match_band(image.get_pixel(x, y)) {
                counts[band] += 1;
            }
        }
    }
    let (best, count) = counts
        .iter()
        .enumerate()
        .max_by_key(|(_, count)| **count)
        .expect("six bands");
    (*count > 0).then_some(best)
}

/// Renders, or skips the case when the machine has no wgpu adapter.
fn try_render(
    engine: &StitchEngine,
    frames: &[RgbaImage],
) -> Option<Vec<(String, RgbaImage)>> {
    let _gpu = gpu_lock();
    match engine.render_frame(frames) {
        Ok(views) => Some(views),
        Err(EngineError::Render(avm_render::RenderError::NoAdapter)) => {
            eprintln!("skipping engine render check: no wgpu adapter available");
            None
        }
        Err(other) => panic!("unexpected render error: {other}"),
    }
}

fn configured_engine(cameras: Vec<Camera>) -> StitchEngine {
    let mut engine = StitchEngine::new();
    engine
        .configure(cameras, &make_car_bowl(), WIDTH, HEIGHT)
        .expect("configure must accept a valid validation fixture");
    engine
}

#[test]
fn configure_n0_is_rejected() {
    let mut engine = StitchEngine::new();
    let error = engine
        .configure(Vec::new(), &make_car_bowl(), WIDTH, HEIGHT)
        .expect_err("zero cameras must be rejected");
    assert!(matches!(error, EngineError::CameraCount(0)), "{error:?}");
}

#[test]
fn configure_n9_is_rejected() {
    let cameras = wrap_kb_side_cameras(&[0.0, 40.0, 80.0, 120.0, 160.0, 200.0, 240.0, 280.0, 320.0]);
    let mut engine = StitchEngine::new();
    let error = engine
        .configure(cameras, &make_car_bowl(), WIDTH, HEIGHT)
        .expect_err("nine cameras must be rejected");
    assert!(matches!(error, EngineError::CameraCount(9)), "{error:?}");
}

#[test]
fn configure_mismatched_image_sizes_is_rejected() {
    let mut cameras = wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]);
    let extrinsics = *cameras[1].extrinsics();
    let model = make_camera_model(
        "kb",
        &CameraModelParams::Kb(KbParams {
            width: 320,
            ..kb_params()
        }),
    )
    .expect("kb fixture must build");
    cameras[1] = Camera::new(extrinsics, model);

    let mut engine = StitchEngine::new();
    let error = engine
        .configure(cameras, &make_car_bowl(), WIDTH, HEIGHT)
        .expect_err("mixed image sizes must be rejected");
    match error {
        EngineError::ImageSizeMismatch { index, .. } => assert_eq!(1, index),
        other => panic!("expected ImageSizeMismatch, got {other:?}"),
    }
}

#[test]
fn configure_null_camera_model_is_rejected() {
    let mut cameras = wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]);
    cameras[2] = Camera::default();

    let mut engine = StitchEngine::new();
    let error = engine
        .configure(cameras, &make_car_bowl(), WIDTH, HEIGHT)
        .expect_err("a null model must be rejected");
    match &error {
        EngineError::MissingCameraModel(index) => assert_eq!(2, *index),
        other => panic!("expected MissingCameraModel, got {other:?}"),
    }
    assert!(
        error.to_string().contains("has no model (Camera.model is null)"),
        "the message must name the null model: {error}"
    );
}

#[test]
fn configure_accepts_one_to_eight_cameras() {
    for n in 1..=8 {
        let azimuths: Vec<f64> = (0..n).map(|i| i as f64 * 360.0 / n as f64).collect();
        let mut engine = StitchEngine::new();
        engine
            .configure(wrap_kb_side_cameras(&azimuths), &make_car_bowl(), WIDTH, HEIGHT)
            .unwrap_or_else(|e| panic!("Configure failed for N={n}: {e}"));
    }
}

#[test]
fn is_configured_is_false_before_configure() {
    assert!(!StitchEngine::new().is_configured());
}

#[test]
fn is_configured_is_true_after_configure() {
    let engine = configured_engine(wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]));
    assert!(engine.is_configured());
}

#[test]
fn num_cameras_reports_the_configured_count() {
    let engine = configured_engine(wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]));
    assert_eq!(4, engine.num_cameras());
}

#[test]
fn render_frame_before_configure_is_rejected() {
    let engine = StitchEngine::new();
    let error = engine
        .render_frame(&[])
        .expect_err("render before configure must be rejected");
    assert!(matches!(error, EngineError::NotConfigured), "{error:?}");
}

#[test]
fn render_frame_with_the_wrong_image_count_is_rejected() {
    let engine = configured_engine(wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]));
    let error = engine
        .render_frame(&colour_frames(3))
        .expect_err("three images for four cameras must be rejected");
    assert!(
        matches!(error, EngineError::ImageCount { expected: 4, got: 3 }),
        "{error:?}"
    );
}

#[test]
fn configure_accepts_an_overridden_bowl_size() {
    let bowl = BowlVariant::from_car_bowl()
        .car_size(18.0, 2.5)
        .grid_step(1.0)
        .outer_radius(25.0)
        .build();
    let mut engine = StitchEngine::new();
    engine
        .configure(
            wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]),
            &bowl,
            WIDTH,
            HEIGHT,
        )
        .expect("a wider bowl is still valid");
    assert!(engine.is_configured());
}

#[test]
fn configure_rejects_a_zero_grid_step() {
    let bowl = BowlVariant::from_car_bowl().grid_step(0.0).build();
    let mut engine = StitchEngine::new();
    let error = engine
        .configure(
            wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]),
            &bowl,
            WIDTH,
            HEIGHT,
        )
        .expect_err("grid_step = 0 must be rejected");
    assert!(matches!(error, EngineError::GridStep(_)), "{error:?}");
}

#[test]
fn configure_rejects_a_non_positive_outer_radius() {
    let bowl = BowlVariant::from_car_bowl().outer_radius(0.0).build();
    let mut engine = StitchEngine::new();
    let error = engine
        .configure(
            wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]),
            &bowl,
            WIDTH,
            HEIGHT,
        )
        .expect_err("outer_radius = 0 must be rejected");
    assert!(matches!(error, EngineError::OuterRadius(_)), "{error:?}");
}

#[test]
fn configure_rejects_a_negative_slope_coefficient() {
    let bowl = BowlVariant::from_car_bowl().slope_coefficient(-0.1).build();
    let mut engine = StitchEngine::new();
    let error = engine
        .configure(
            wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]),
            &bowl,
            WIDTH,
            HEIGHT,
        )
        .expect_err("a negative slope must be rejected");
    assert!(matches!(error, EngineError::SlopeCoefficient(_)), "{error:?}");
}

#[test]
fn configure_rejects_a_negative_height_exponent() {
    let bowl = BowlVariant::from_car_bowl().height_exponent(-1.0).build();
    let mut engine = StitchEngine::new();
    let error = engine
        .configure(
            wrap_kb_side_cameras(&[0.0, 90.0, 180.0, 270.0]),
            &bowl,
            WIDTH,
            HEIGHT,
        )
        .expect_err("a negative height exponent must be rejected");
    assert!(matches!(error, EngineError::HeightExponent(_)), "{error:?}");
}

#[test]
fn public_api_headers_are_self_contained() {
    let bowl = BowlParams::for_car_size(5.6, 2.2, 2.9);
    assert_eq!(12.6, bowl.outer_radius());
    assert_eq!(4, bowl.default_view_poses().len());
}

#[test]
fn render_frame_returns_four_named_views() {
    let engine = configured_engine(wrap_kb_cameras(
        &vec![kb_params(); 4],
        &make_look_at_cameras(4),
    ));
    let Some(views) = try_render(&engine, &colour_frames(4)) else {
        return;
    };
    assert_eq!(4, views.len());
    let names: Vec<&str> = views.iter().map(|(name, _)| name.as_str()).collect();
    assert_eq!(
        vec!["3d_front", "3d_rear", "3d_left", "3d_right"],
        names
    );
    let hashes: Vec<u64> = views.iter().map(|(_, image)| content_hash(image)).collect();
    for first in 0..hashes.len() {
        for second in first + 1..hashes.len() {
            assert_ne!(
                hashes[first], hashes[second],
                "views {} and {} rendered identically",
                names[first], names[second]
            );
        }
    }
}

#[test]
fn configure_kb_path_renders_four_views() {
    let engine = configured_engine(wrap_kb_cameras(
        &vec![kb_params(); 4],
        &make_look_at_cameras(4),
    ));
    assert!(engine.is_configured());
    let Some(views) = try_render(&engine, &colour_frames(4)) else {
        return;
    };
    assert_eq!(4, views.len());
}

#[test]
fn render_frame_output_size_matches_the_engine_default() {
    let engine = configured_engine(wrap_kb_cameras(
        &vec![kb_params(); 4],
        &make_look_at_cameras(4),
    ));
    let Some(views) = try_render(&engine, &colour_frames(4)) else {
        return;
    };
    for (name, image) in &views {
        assert_eq!((WIDTH, HEIGHT), image.dimensions(), "{name}");
    }
}

#[test]
fn render_frame_output_is_not_black() {
    let engine = configured_engine(wrap_kb_cameras(
        &vec![kb_params(); 4],
        &make_look_at_cameras(4),
    ));
    let Some(views) = try_render(&engine, &colour_frames(4)) else {
        return;
    };
    for (name, image) in &views {
        assert!(count_vivid_pixels(image) > 0, "{name} is entirely dull");
    }
}

#[test]
fn render_frame_with_eight_cameras() {
    let engine = configured_engine(wrap_kb_cameras(
        &vec![kb_params(); 8],
        &make_look_at_cameras(8),
    ));
    let Some(views) = try_render(&engine, &colour_frames(8)) else {
        return;
    };
    assert_eq!(4, views.len());
    let vivid: usize = views.iter().map(|(_, image)| count_vivid_pixels(image)).sum();
    assert!(vivid > 0);
}

#[test]
fn front_view_is_not_vertically_flipped() {
    let bowl = BowlVariant::from_car_bowl().car_center([0.0, 0.0]).build();
    let mut engine = StitchEngine::new();
    engine
        .configure(
            wrap_kb_cameras(
                &[kb_params()],
                &[make_look_at_camera(0.0, 4.0, 2.0, 0.0)],
            ),
            &bowl,
            WIDTH,
            HEIGHT,
        )
        .expect("configure");
    let Some(views) = try_render(&engine, &[band_frame()]) else {
        return;
    };
    let front = &views
        .iter()
        .find(|(name, _)| name == "3d_front")
        .expect("front view")
        .1;
    assert_eq!(
        Some(1),
        dominant_band(front, 60, 240),
        "upper rows must sample camera image band 1"
    );
    assert_eq!(
        Some(4),
        dominant_band(front, 400, 600),
        "bottom rows must sample camera image band 4"
    );
}

