//! These assertions are deliberately manifest-level: the dependency direction is
//! part of the contract, and a plain `cargo build` would accept a stray `opencv`
//! or `wgpu` dependency in the CPU core.

const CORE_MANIFEST: &str = include_str!("../Cargo.toml");
const RENDER_MANIFEST: &str = include_str!("../../avm-render/Cargo.toml");

fn normalized(manifest: &str) -> String {
    manifest.to_lowercase()
}

#[test]
fn core_manifest_has_no_gpu_opencv_or_osg_dependency() {
    let manifest = normalized(CORE_MANIFEST);
    for forbidden in ["opencv", "osg", "wgpu", "naga", "image"] {
        assert!(
            !manifest.contains(forbidden),
            "avm-core must not depend on {forbidden} (A2)"
        );
    }
}

#[test]
fn core_manifest_declares_the_mandated_dependencies() {
    let manifest = normalized(CORE_MANIFEST);
    for required in ["nalgebra", "serde", "serde_json", "thiserror"] {
        assert!(
            manifest.contains(required),
            "avm-core must depend on {required} (A2)"
        );
    }
}

#[test]
fn render_manifest_depends_on_core_and_wgpu_only_for_gpu_work() {
    let manifest = normalized(RENDER_MANIFEST);
    assert!(manifest.contains("avm-core"), "avm-render must use avm-core");
    assert!(manifest.contains("wgpu"), "avm-render owns the wgpu dependency");
    assert!(!manifest.contains("opencv"), "avm-render must not use OpenCV");
}

/// The CPU core must not reach for the render crate (that would drag wgpu into
/// the algorithm crate through the back door).
#[test]
fn core_manifest_does_not_depend_on_the_render_crate() {
    let manifest = normalized(CORE_MANIFEST);
    assert!(
        !manifest.contains("avm-render"),
        "avm-core must not depend on avm-render (A1)"
    );
}

/// Resolves the real dependency closure of `package` and returns the crate
/// names, so a leak through an intermediate crate is caught even when the
/// manifest looks clean.
fn dependency_names(package: &str) -> Vec<String> {
    let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    let output = std::process::Command::new(cargo)
        .args(["tree", "-p", package, "--prefix", "none", "--offline"])
        .current_dir(&workspace_root)
        .output()
        .expect("cargo tree must be runnable from the workspace root");
    assert!(
        output.status.success(),
        "cargo tree -p {package} failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8_lossy(&output.stdout)
        .to_lowercase()
        .lines()
        .filter_map(|line| line.split_whitespace().next().map(str::to_string))
        .collect()
}

/// Belt and braces: the CPU core must not reach the GPU stack directly or
/// transitively (`avm-core -> foo -> wgpu`).
#[test]
fn core_dependency_tree_has_no_gpu_or_image_crates() {
    let names = dependency_names("avm-core");
    for forbidden in ["wgpu", "naga", "image", "opencv", "osg", "avm-render"] {
        assert!(
            !names.iter().any(|name| name == forbidden),
            "avm-core dependency tree contains {forbidden} (A1/A2): {names:?}"
        );
    }
}

/// A1: statistics stay CPU-only. The window lives in `avm-viewer`, so neither
/// the GPU stack nor the viewer may appear in the CLI closure; checking the
/// closure (not the manifest text) catches an indirect leak.
#[test]
fn cli_dependency_tree_has_no_gpu_or_viewer_crates() {
    let names = dependency_names("avm-cli");
    for forbidden in ["wgpu", "winit", "naga", "avm-render", "avm-viewer"] {
        assert!(
            !names.iter().any(|name| name == forbidden),
            "avm-cli dependency tree contains {forbidden} (A1): {names:?}"
        );
    }
    assert!(
        names.iter().any(|name| name == "avm-core"),
        "avm-cli must depend on avm-core: {names:?}"
    );
}
