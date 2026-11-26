#![cfg(feature = "render")]
//! Pixel parity with the reference images; the fixtures are external ground
//! truth described in `fixtures/parity/data_4/METADATA.md`. Without a GPU
//! adapter the test fails rather than skipping. `AVM_PARITY_DATA_DIR` overrides
//! only the rendered dataset, never the reference.

use std::path::{Path, PathBuf};

use avm_stitch::bowl::BowlModel;
use avm_stitch::calibration::Calibration;
use avm_stitch::render::{
    load_camera_image, render_all_views_with, RenderError, RenderSize, SceneData, ViewSpec,
    BACKGROUND_CLEAR,
};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 640;
const PSNR_MIN_DB: f64 = 45.0;
const MEAN_DIFF_MAX: f64 = 1.0;

fn workspace_path(relative: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join(relative)
}

fn data_dir() -> PathBuf {
    std::env::var("AVM_PARITY_DATA_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| workspace_path("examples/data_4"))
}

fn fixture_dir() -> PathBuf {
    workspace_path("tests/fixtures/parity/data_4")
}

/// VCS -> y-up render basis, the same remap the renderer applies.
fn to_render_basis(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[2] as f32, -p[1] as f32]
}

fn render_data_dir(data_dir: &Path) -> (Vec<String>, Vec<Vec<u8>>) {
    let calibration = Calibration::load_from_path(data_dir.join("calibration.json"))
        .expect("calibration.json must load");
    let bowl = calibration.bowl_params();
    let model = BowlModel::new(&bowl, &calibration.cameras);

    let images: Vec<_> = calibration
        .image_files
        .iter()
        .map(|name| load_camera_image(data_dir.join(name)).expect("camera image must load"))
        .collect();
    let scene = SceneData::from_bowl_model(&model).with_camera_images(images);

    let poses = bowl.default_view_poses();
    let max_eye = poses
        .iter()
        .map(|pose| {
            let d = [
                pose.position[0] - pose.target[0],
                pose.position[1] - pose.target[1],
                pose.position[2] - pose.target[2],
            ];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        })
        .fold(0.0f64, f64::max);
    let clip = bowl.clip_planes_for(max_eye);

    let names: Vec<String> = poses.iter().map(|pose| pose.name.clone()).collect();
    let views: Vec<ViewSpec> = poses
        .iter()
        .map(|pose| {
            ViewSpec::look_at(
                to_render_basis(pose.position),
                to_render_basis(pose.target),
                to_render_basis(pose.up),
                pose.fovy_deg as f32,
                WIDTH as f32 / HEIGHT as f32,
                clip.near as f32,
                clip.far as f32,
            )
        })
        .collect();

    let frames = render_all_views_with(&scene, &views, RenderSize::new(WIDTH, HEIGHT), BACKGROUND_CLEAR)
        .unwrap_or_else(|error| match error {
            RenderError::NoAdapter => panic!("parity test requires a GPU adapter"),
            other => panic!("parity render failed: {other}"),
        });
    (names, frames)
}

/// Per-channel PSNR in dB over the whole frame.
fn channel_psnr(reference: &[u8], actual: &[u8], channel: usize) -> f64 {
    let mut squared_error = 0.0f64;
    let mut count = 0u64;
    for (a, b) in reference.chunks_exact(4).zip(actual.chunks_exact(4)) {
        let d = a[channel] as f64 - b[channel] as f64;
        squared_error += d * d;
        count += 1;
    }
    let mse = squared_error / count as f64;
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0 * 255.0 / mse).log10()
    }
}

/// Mean of the signed difference (reference - actual), per channel.
fn mean_diff(reference: &[u8], actual: &[u8], channel: usize) -> f64 {
    let mut sum = 0.0f64;
    let mut count = 0u64;
    for (a, b) in reference.chunks_exact(4).zip(actual.chunks_exact(4)) {
        sum += a[channel] as f64 - b[channel] as f64;
        count += 1;
    }
    sum / count as f64
}

fn write_artifacts(view: &str, actual: &[u8], reference: &[u8]) {
    let root = workspace_path("target/parity");
    std::fs::create_dir_all(&root).expect("create target/parity");
    image::save_buffer(
        root.join(format!("{view}_rust.png")),
        actual,
        WIDTH,
        HEIGHT,
        image::ColorType::Rgba8,
    )
    .expect("write rust frame");

    let diff: Vec<u8> = reference
        .chunks_exact(4)
        .zip(actual.chunks_exact(4))
        .flat_map(|(a, b)| {
            let d = |i: usize| (a[i] as i16 - b[i] as i16).unsigned_abs().min(255) as u8;
            [d(0), d(1), d(2), 255]
        })
        .collect();
    image::save_buffer(
        root.join(format!("{view}_diff.png")),
        &diff,
        WIDTH,
        HEIGHT,
        image::ColorType::Rgba8,
    )
    .expect("write diff frame");
}

#[test]
fn each_default_view_matches_the_cpp_reference() {
    let data_dir = data_dir();
    let fixture_dir = fixture_dir();
    println!("data: {}", data_dir.display());
    println!("reference: {}", fixture_dir.display());

    let (names, frames) = render_data_dir(&data_dir);
    assert_eq!(4, names.len(), "the default presets must produce four views");
    assert_eq!(
        names.len(),
        frames.len(),
        "the renderer must return one frame per view"
    );
    assert_eq!(4, frames.len(), "the renderer must produce four frames");

    let mut worst = f64::INFINITY;
    let mut failures = Vec::new();
    for (name, frame) in names.iter().zip(&frames) {
        assert_eq!(
            (WIDTH * HEIGHT * 4) as usize,
            frame.len(),
            "{name}: frame size"
        );

        let reference_path = fixture_dir.join(format!("{name}.png"));
        let reference = image::open(&reference_path)
            .unwrap_or_else(|e| panic!("missing reference {}: {e}", reference_path.display()))
            .to_rgba8();
        assert_eq!((WIDTH, HEIGHT), reference.dimensions(), "{name}: reference size");
        let reference = reference.as_raw();

        let mut view_psnr = [f64::INFINITY; 3];
        let mut view_mean = [0.0f64; 3];
        for channel in 0..3 {
            view_psnr[channel] = channel_psnr(reference, frame, channel);
            view_mean[channel] = mean_diff(reference, frame, channel);
            worst = worst.min(view_psnr[channel]);
        }
        println!(
            "{name:9} PSNR R={:.2} G={:.2} B={:.2} dB  mean dR={:.3} dG={:.3} dB={:.3}",
            view_psnr[0], view_psnr[1], view_psnr[2], view_mean[0], view_mean[1], view_mean[2]
        );

        for channel in 0..3 {
            if view_psnr[channel] <= PSNR_MIN_DB {
                failures.push(format!(
                    "{name}: channel {channel} PSNR {:.2} dB <= {PSNR_MIN_DB}",
                    view_psnr[channel]
                ));
            }
            if view_mean[channel].abs() >= MEAN_DIFF_MAX {
                failures.push(format!(
                    "{name}: channel {channel} mean difference {:.3} >= {MEAN_DIFF_MAX}",
                    view_mean[channel]
                ));
            }
        }
        if failures.iter().any(|f| f.starts_with(name)) {
            write_artifacts(name, frame, reference);
        }
    }

    assert!(
        failures.is_empty(),
        "parity failures (artifacts in target/parity):\n{}",
        failures.join("\n")
    );
    println!("worst channel PSNR: {worst:.2} dB");
}
