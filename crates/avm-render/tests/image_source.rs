//! Camera image loading through the `image` crate (AC A2).

use std::path::PathBuf;

use avm_render::image_source::load_camera_image;

fn examples_data(set: &str, file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples")
        .join(set)
        .join(file)
}

#[test]
fn loads_a_real_camera_frame_as_rgba8() {
    let image = load_camera_image(examples_data("data_4", "front.png")).expect("front.png must load");
    assert_eq!(640, image.width());
    assert_eq!(480, image.height());
    // Pin the channel order: the frozen fixture's (182, 224) pixel is RGBA.
    assert_eq!([124, 124, 188, 255], image.get_pixel(182, 224).0);
    // The fixture frames are not uniformly black.
    assert!(image.pixels().any(|p| p[0] > 0 || p[1] > 0 || p[2] > 0));
}

#[test]
fn missing_image_reports_the_path() {
    let err = load_camera_image(examples_data("data_4", "missing.png")).expect_err("must fail");
    assert!(err.to_string().contains("missing.png"), "got: {err}");
}
