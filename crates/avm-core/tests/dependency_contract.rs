//! Architectural contract for the Rust workspace (batch B1, AC A1/A2).
//!
//! These assertions are deliberately manifest-level: the dependency direction is
//! part of the port's contract, and a plain `cargo build` would happily accept a
//! stray `opencv` or `wgpu` dependency in the CPU core.

const CORE_MANIFEST: &str = include_str!("../Cargo.toml");
const RENDER_MANIFEST: &str = include_str!("../../avm-render/Cargo.toml");
const CLI_MANIFEST: &str = include_str!("../../avm-cli/Cargo.toml");

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

/// Belt and braces: resolve the real dependency closure, so a transitive leak
/// (`avm-core -> foo -> wgpu`) is caught even when `avm-core/Cargo.toml` looks
/// clean.
#[test]
fn core_dependency_tree_has_no_gpu_or_image_crates() {
    let workspace_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    let output = std::process::Command::new(cargo)
        .args(["tree", "-p", "avm-core", "--prefix", "none", "--offline"])
        .current_dir(&workspace_root)
        .output()
        .expect("cargo tree must be runnable from the workspace root");
    assert!(
        output.status.success(),
        "cargo tree -p avm-core failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );

    let tree = String::from_utf8_lossy(&output.stdout).to_lowercase();
    let names: Vec<&str> = tree
        .lines()
        .filter_map(|line| line.split_whitespace().next())
        .collect();
    for forbidden in ["wgpu", "naga", "image", "opencv", "osg", "avm-render"] {
        assert!(
            !names.contains(&forbidden),
            "avm-core dependency tree contains {forbidden} (A1/A2)\n{tree}"
        );
    }
}

#[test]
fn cli_manifest_stays_cpu_only() {
    let manifest = normalized(CLI_MANIFEST);
    assert!(manifest.contains("avm-core"), "avm-cli must use avm-core");
    assert!(
        !manifest.contains("wgpu"),
        "avm-cli must not link the GPU stack for statistics"
    );
    assert!(
        !manifest.contains("avm-render"),
        "avm-cli must not depend on avm-render (it would pull in wgpu)"
    );
}
