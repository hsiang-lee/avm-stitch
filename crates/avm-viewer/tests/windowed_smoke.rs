//! Drives the windowed entry in a child process: two frames must be presented
//! and the screenshot must match the offscreen render of the same view byte for
//! byte. The window contents are not judged visually, only this capture.

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Output, Stdio};
use std::time::{Duration, Instant};

use avm_core::bowl::BowlModel;
use avm_core::calibration::Calibration;
use avm_render::{
    load_camera_image, render_all_views_with, RenderSize, SceneData, BACKGROUND_CLEAR,
};
use avm_viewer::OrbitController;

/// Extracts the presented surface size from the viewer's report
/// `Presented N frame(s) at WxH`. A run that presented no frame reports no
/// size, so `None` here is a test failure, not a reason to fall back.
fn presented_size(stdout: &str) -> Option<(u32, u32)> {
    let marker = " frame(s) at ";
    let rest = stdout.split_once(marker)?.1;
    let token = rest.split_whitespace().next()?;
    let (width, height) = token.split_once('x')?;
    Some((width.parse().ok()?, height.parse().ok()?))
}

fn display_available() -> bool {
    std::env::var_os("DISPLAY").is_some() || std::env::var_os("WAYLAND_DISPLAY").is_some()
}

fn xvfb_available() -> bool {
    Command::new("sh")
        .args(["-c", "command -v xvfb-run"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}

/// Waits for `child` with a deadline so a stuck event loop fails the test
/// instead of hanging the run.
fn wait_with_deadline(mut child: Child, seconds: u64) -> Output {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("poll the viewer process") {
            Some(_) => return child.wait_with_output().expect("collect viewer output"),
            None if start.elapsed() > Duration::from_secs(seconds) => {
                let _ = child.kill();
                panic!("the windowed viewer did not exit within {seconds}s");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/data_4")
}

/// Renders the same camera the viewer starts with, offscreen, at the same size.
fn reference_frame(data_dir: &Path, size: RenderSize) -> Vec<u8> {
    let calibration = Calibration::load_from_path(data_dir.join("calibration.json"))
        .expect("calibration.json must load");
    let bowl = calibration.bowl_params();
    let model = BowlModel::new(&bowl, &calibration.cameras);
    let images = calibration
        .image_files
        .iter()
        .map(|name| load_camera_image(data_dir.join(name)))
        .collect::<Result<Vec<_>, _>>()
        .expect("camera images must load");
    let scene = SceneData::from_bowl_model(&model).with_camera_images(images);
    let controller = OrbitController::from_home(&bowl);
    let view = controller.view_spec(size.width as f32 / size.height as f32);
    render_all_views_with(
        &scene,
        &[view],
        RenderSize::new(size.width, size.height),
        BACKGROUND_CLEAR,
    )
    .expect("the viewer just created a device, so the reference render must succeed")
    .into_iter()
    .next()
    .expect("one view was requested")
}

#[test]
fn windowed_viewer_draws_two_frames_and_exits() {
    let data_dir = data_dir();
    let screenshot = std::env::temp_dir().join(format!(
        "avm-viewer-smoke-{}.png",
        std::process::id()
    ));
    let _ = std::fs::remove_file(&screenshot);
    let binary = env!("CARGO_BIN_EXE_avm-viewer");
    let arguments: Vec<String> = vec![
        "--frames".into(),
        "2".into(),
        "--screenshot".into(),
        screenshot.display().to_string(),
        data_dir.display().to_string(),
    ];

    let mut command = if display_available() {
        let mut command = Command::new(binary);
        command.args(&arguments);
        command
    } else if xvfb_available() {
        let mut command = Command::new("xvfb-run");
        command.arg("-a").arg(binary).args(&arguments);
        command
    } else {
        eprintln!("skipping windowed smoke test: no DISPLAY/WAYLAND_DISPLAY and no xvfb-run");
        return;
    };

    let output = wait_with_deadline(
        command
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("spawn the windowed viewer"),
        120,
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "windowed viewer failed ({:?})\nstdout:\n{stdout}\nstderr:\n{stderr}",
        output.status
    );
    assert!(
        stdout.contains("Presented 2 frame(s)"),
        "the viewer must report two presented frames, got:\n{stdout}"
    );
    let lowered = stderr.to_lowercase();
    assert!(
        !lowered.contains("validation error"),
        "wgpu reported a validation error:\n{stderr}"
    );

    let (width, height) = presented_size(&stdout).unwrap_or_else(|| {
        panic!("a presented run must report its surface size as `at WxH`, got:\n{stdout}")
    });

    let image = image::open(&screenshot)
        .expect("the requested screenshot must exist")
        .to_rgba8();
    assert_eq!(
        (width, height),
        image.dimensions(),
        "unexpected screenshot size"
    );
    let reference = reference_frame(&data_dir, RenderSize::new(width, height));
    let actual = image.as_raw();
    assert_eq!(
        reference.len(),
        actual.len(),
        "screenshot and reference differ in length"
    );
    let differing = actual
        .iter()
        .zip(&reference)
        .filter(|(captured, expected)| captured != expected)
        .count();
    assert_eq!(
        0, differing,
        "the screenshot differs from the offscreen render in {differing} bytes"
    );
    let _ = std::fs::remove_file(&screenshot);
}
