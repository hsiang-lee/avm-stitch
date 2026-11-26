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

#[test]
fn cli_manifest_stays_cpu_only() {
    let manifest = normalized(CLI_MANIFEST);
    assert!(manifest.contains("avm-core"), "avm-cli must use avm-core");
    assert!(
        !manifest.contains("wgpu"),
        "avm-cli must not link the GPU stack for statistics"
    );
}
