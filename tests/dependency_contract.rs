//! Dependency direction: the CPU-only build must not reach the GPU or image
//! stack or the render sources, and each feature must pull in exactly the
//! modules it advertises.

use std::path::PathBuf;
use std::process::Command;
use std::sync::Mutex;

/// Cargo serialises on the package cache anyway; this also stops two tests from
/// flipping features back and forth and recompiling the library twice.
static CARGO_LOCK: Mutex<()> = Mutex::new(());

fn cargo_lock() -> std::sync::MutexGuard<'static, ()> {
    CARGO_LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn cargo() -> String {
    std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string())
}

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn run_cargo(arguments: &[&str]) -> std::process::Output {
    let output = Command::new(cargo())
        .args(arguments)
        .current_dir(manifest_dir())
        .output()
        .expect("cargo must be runnable from the package root");
    assert!(
        output.status.success(),
        "cargo {arguments:?} failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    output
}

/// First token of every `cargo tree --prefix none` line: the resolved crates.
///
/// Dev-dependencies are included on purpose: the point of the test is that the
/// GPU stack is not reachable at all, not merely that it is not linked.
fn dependency_names(feature_arguments: &[&str]) -> Vec<String> {
    let mut arguments = vec!["tree", "--prefix", "none", "--offline"];
    arguments.extend(feature_arguments);
    let output = run_cargo(&arguments);
    String::from_utf8_lossy(&output.stdout)
        .to_lowercase()
        .lines()
        .filter_map(|line| line.split_whitespace().next().map(str::to_string))
        .collect()
}

/// Compiles the library with the given feature arguments and returns the
/// dep-info file listing every source rustc read, so feature gating is checked
/// on the actual build graph rather than on the manifest.
fn compiled_sources(label: &str, feature_arguments: &[&str]) -> String {
    let dep_info =
        std::env::temp_dir().join(format!("avm-stitch-{label}-{}.d", std::process::id()));
    let _ = std::fs::remove_file(&dep_info);
    let emit = format!("--emit=dep-info={}", dep_info.display());
    let mut arguments = vec!["rustc", "--lib"];
    arguments.extend(feature_arguments);
    arguments.extend(["--", &emit]);
    run_cargo(&arguments);
    let sources = std::fs::read_to_string(&dep_info)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", dep_info.display()));
    let _ = std::fs::remove_file(&dep_info);
    sources
}

#[test]
fn cpu_only_build_does_not_reach_the_gpu_stack() {
    let _cargo = cargo_lock();
    let names = dependency_names(&["--no-default-features"]);
    for forbidden in ["wgpu", "winit", "naga", "image"] {
        assert!(
            !names.iter().any(|name| name == forbidden),
            "--no-default-features still resolves {forbidden}: {names:?}"
        );
    }
}

#[test]
fn default_build_does_reach_the_render_stack() {
    let _cargo = cargo_lock();
    let names = dependency_names(&[]);
    for expected in ["wgpu", "winit", "naga", "image"] {
        assert!(
            names.iter().any(|name| name == expected),
            "the default features must resolve {expected}: {names:?}"
        );
    }
}

#[test]
fn no_sibling_crates_remain_in_the_closure() {
    let _cargo = cargo_lock();
    let names = dependency_names(&["--no-default-features"]);
    for sibling in ["avm-core", "avm-render", "avm-engine", "avm-viewer", "avm-cli"] {
        assert!(
            !names.iter().any(|name| name == sibling),
            "the workspace must be one package, found {sibling}: {names:?}"
        );
    }
    assert!(
        names.iter().any(|name| name == "avm-stitch"),
        "cargo tree must describe the package itself: {names:?}"
    );
}

#[test]
fn cpu_only_build_reads_no_render_sources() {
    let _cargo = cargo_lock();
    let sources = compiled_sources("cpu", &["--no-default-features"]);
    assert!(
        !sources.contains("src/render") && !sources.contains("src/engine.rs"),
        "the CPU-only build must not read src/render or src/engine.rs:\n{sources}"
    );
    assert!(
        sources.contains("src/bowl") && sources.contains("src/camera"),
        "the CPU-only build must still read the algorithm modules:\n{sources}"
    );
}

#[test]
fn default_features_read_both_render_and_window_sources() {
    let _cargo = cargo_lock();
    let sources = compiled_sources("default", &[]);
    assert!(
        sources.contains("src/render/renderer.rs") && sources.contains("src/engine.rs"),
        "the default build must read the render and engine sources:\n{sources}"
    );
    assert!(
        sources.contains("src/render/windowed.rs"),
        "the viewer feature is on by default and must compile the window:\n{sources}"
    );
}

#[test]
fn render_feature_reads_the_render_sources_but_not_the_window() {
    let _cargo = cargo_lock();
    let sources = compiled_sources("render", &["--no-default-features", "--features", "render"]);
    assert!(
        sources.contains("src/render/renderer.rs") && sources.contains("src/engine.rs"),
        "the render feature must compile the render and engine sources:\n{sources}"
    );
    assert!(
        !sources.contains("src/render/windowed.rs"),
        "the render feature alone must not compile the window:\n{sources}"
    );
}

#[test]
fn viewer_feature_implies_the_render_sources() {
    let _cargo = cargo_lock();
    // A `viewer` that stopped implying `render` would fail to compile here
    // instead of silently shipping a window without a renderer.
    let sources = compiled_sources("viewer", &["--no-default-features", "--features", "viewer"]);
    assert!(
        sources.contains("src/render/windowed.rs") && sources.contains("src/render/renderer.rs"),
        "the viewer feature must pull in the render sources:\n{sources}"
    );
}

#[test]
fn cpu_only_build_compiles_the_cli_binary() {
    let _cargo = cargo_lock();
    run_cargo(&[
        "build",
        "--quiet",
        "--no-default-features",
        "--bin",
        "avm-cli",
    ]);
}
