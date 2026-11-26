//! Drives the windowed entry in a child process for two frames. This proves the
//! event loop starts, presents and exits cleanly; it cannot judge the pixels.

use std::process::{Child, Command, Output, Stdio};
use std::time::{Duration, Instant};

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

#[test]
fn windowed_viewer_draws_two_frames_and_exits() {
    let data_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/data_4");
    let binary = env!("CARGO_BIN_EXE_avm-viewer");
    let viewer_args = [
        std::ffi::OsString::from("--frames"),
        std::ffi::OsString::from("2"),
        data_dir.into_os_string(),
    ];

    let mut command = if display_available() {
        let mut command = Command::new(binary);
        command.args(&viewer_args);
        command
    } else if xvfb_available() {
        let mut command = Command::new("xvfb-run");
        command.arg("-a").arg(binary).args(&viewer_args);
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
        stdout.contains("Presented 2"),
        "the viewer must report the frames it presented, got:\n{stdout}"
    );
    let lowered = stderr.to_lowercase();
    assert!(
        !lowered.contains("validation error"),
        "wgpu reported a validation error:\n{stderr}"
    );
}
