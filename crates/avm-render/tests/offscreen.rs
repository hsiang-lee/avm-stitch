//! Offscreen render skeleton: returns frames, and returns an error (never
//! panics) when no GPU adapter exists (AC A7).
//!
//! The colour-sensitive cases build a single screen-covering triangle so that
//! the assertion targets the shader's per-layer weight gate rather than the
//! bake result; tests skip (not fail) on machines without a GPU adapter.

use avm_core::bowl::{BowlModel, BowlParams};
use avm_core::camera::{make_camera_model, Camera, CameraExtrinsics, CameraModelParams, KbParams};
use avm_render::vertex::PackedVertex;
use avm_render::{render_all_views, RenderError, SceneData, ViewSpec};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

fn bowl_scene() -> SceneData {
    let params = BowlParams::new(2.0, 1.0, 0.5, 0.4, 4.0, 0.1, 2.2, 4.0, [0.0, 0.0]);
    let intrinsics = KbParams {
        width: 64,
        height: 48,
        fx: 40.0,
        fy: 40.0,
        cx: 32.0,
        cy: 24.0,
        ..Default::default()
    };
    let cameras: Vec<Camera> = [0.0f64, 90.0, 180.0, 270.0]
        .into_iter()
        .map(|az| {
            let rad = az.to_radians();
            let extrinsics = CameraExtrinsics::from_rows(
                [
                    [rad.cos(), 0.0, rad.sin()],
                    [0.0, 1.0, 0.0],
                    [-rad.sin(), 0.0, rad.cos()],
                ],
                [4.0 * rad.cos(), 4.0 * rad.sin(), 2.0],
            );
            let model = make_camera_model("kb", &CameraModelParams::Kb(intrinsics.clone()))
                .expect("kb model");
            Camera::new(extrinsics, model)
        })
        .collect();

    let model = BowlModel::new(&params, &cameras);
    let images = (0..4)
        .map(|i| {
            image::RgbaImage::from_pixel(
                64,
                48,
                image::Rgba([(40 * (i + 1)) as u8, 20, 200, 255]),
            )
        })
        .collect();
    SceneData::from_bowl_model(&model).with_camera_images(images)
}

/// Screen-covering triangle in the y-up render basis, seen from above.
fn triangle_scene(weight: [f32; 4], num_cameras: usize) -> SceneData {
    let mut vertices = [PackedVertex::default(); 3];
    vertices[0].position = [-10.0, 0.0, -10.0];
    vertices[1].position = [10.0, 0.0, -10.0];
    vertices[2].position = [0.0, 0.0, 10.0];
    for vertex in &mut vertices {
        vertex.uv[0] = [0.5, 0.5, 0.0, 0.0];
        vertex.weights[0] = weight;
    }
    SceneData {
        vertices: vertices.to_vec(),
        indices: vec![0, 1, 2],
        num_cameras,
        camera_images: vec![image::RgbaImage::from_pixel(
            64,
            48,
            image::Rgba([40, 20, 200, 255]),
        )],
    }
}

fn top_down_view() -> ViewSpec {
    ViewSpec::look_at(
        [0.0, 14.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        60.0,
        WIDTH as f32 / HEIGHT as f32,
        0.1,
        100.0,
    )
}

/// Renders one frame, or returns `None` when the machine has no adapter.
fn render_single(scene: &SceneData) -> Option<Vec<u8>> {
    match render_all_views(scene, &[top_down_view()], WIDTH, HEIGHT) {
        Ok(mut frames) => Some(frames.remove(0)),
        Err(RenderError::NoAdapter) => {
            eprintln!("skipping offscreen check: no wgpu adapter available");
            None
        }
        Err(other) => panic!("unexpected render error: {other}"),
    }
}

fn center_pixel(frame: &[u8]) -> [u8; 4] {
    let x = WIDTH / 2;
    let y = HEIGHT / 2;
    let offset = ((y * WIDTH + x) * 4) as usize;
    [
        frame[offset],
        frame[offset + 1],
        frame[offset + 2],
        frame[offset + 3],
    ]
}

#[test]
fn renders_every_view_or_skips_without_adapter() {
    let scene = bowl_scene();
    let views = vec![top_down_view(), top_down_view()];

    match render_all_views(&scene, &views, WIDTH, HEIGHT) {
        Ok(frames) => {
            assert_eq!(views.len(), frames.len());
            for frame in frames {
                assert_eq!((WIDTH * HEIGHT * 4) as usize, frame.len());
                // The shader always writes an opaque result, and the clear is opaque.
                assert!(frame.chunks_exact(4).all(|px| px[3] == 255));
            }
        }
        Err(RenderError::NoAdapter) => {
            eprintln!("skipping offscreen render: no wgpu adapter available");
        }
        Err(other) => panic!("unexpected render error: {other}"),
    }
}

#[test]
fn weighted_camera_layer_color_reaches_the_framebuffer() {
    let scene = triangle_scene([1.0, 0.0, 0.0, 0.0], 1);
    let Some(frame) = render_single(&scene) else {
        return;
    };
    let [r, g, b, a] = center_pixel(&frame);
    assert_eq!([40, 20, 200, 255], [r, g, b, a]);
}

#[test]
fn weight_scales_the_sampled_layer_color() {
    let scene = triangle_scene([0.5, 0.0, 0.0, 0.0], 1);
    let Some(frame) = render_single(&scene) else {
        return;
    };
    let [r, g, b, _] = center_pixel(&frame);
    assert!((r as i32 - 20).abs() <= 1, "r = {r}");
    assert!((g as i32 - 10).abs() <= 1, "g = {g}");
    assert!((b as i32 - 100).abs() <= 1, "b = {b}");
}

#[test]
fn weights_below_the_hard_gate_are_ignored() {
    // 0.0009 < WEIGHT_MIN (0.001): the layer must not contribute anything.
    let scene = triangle_scene([0.0009, 0.0, 0.0, 0.0], 1);
    let Some(frame) = render_single(&scene) else {
        return;
    };
    assert_eq!([0, 0, 0, 255], center_pixel(&frame));
}

#[test]
fn zero_camera_count_disables_every_layer() {
    let scene = triangle_scene([1.0, 0.0, 0.0, 0.0], 0);
    let Some(frame) = render_single(&scene) else {
        return;
    };
    assert_eq!([0, 0, 0, 255], center_pixel(&frame));
}

#[test]
fn empty_scene_is_rejected_without_touching_the_gpu() {
    let scene = SceneData {
        vertices: Vec::new(),
        indices: Vec::new(),
        num_cameras: 0,
        camera_images: Vec::new(),
    };
    let err = render_all_views(&scene, &[top_down_view()], WIDTH, HEIGHT)
        .err()
        .expect("empty scene must fail");
    assert!(matches!(err, RenderError::EmptyScene));
}
