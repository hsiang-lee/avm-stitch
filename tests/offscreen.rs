#![cfg(feature = "render")]
//! Without a GPU adapter these tests skip instead of failing. The colour cases
//! use a screen-covering triangle so the assertion isolates the shader weight
//! gate; the bowl case uses cameras that actually look at the bowl.

use avm_stitch::bowl::{BowlModel, BowlParams};
use avm_stitch::camera::{make_camera_model, Camera, CameraExtrinsics, CameraModelParams, KbParams};
use avm_stitch::{Matrix3, Vector3};
use avm_stitch::render::vertex::{PackedVertex, MAX_CAMERAS};
use avm_stitch::render::{render_all_views, render_all_views_with, RenderError, RenderSize, SceneData, ViewSpec};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

/// llvmpipe segfaults when several tests create devices at the same time, so the
/// GPU work in this binary is serialised.
static GPU_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn gpu_lock() -> std::sync::MutexGuard<'static, ()> {
    GPU_LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Camera basis columns are `(right, down, forward)`, in that order.
fn look_at_extrinsics(az_deg: f64, radius: f64, height: f64, target_z: f64) -> CameraExtrinsics {
    let rad = az_deg.to_radians();
    let pos = Vector3::new(radius * rad.cos(), radius * rad.sin(), height);
    let target = Vector3::new(0.0, 0.0, target_z);
    let forward = (target - pos).normalize();
    let world_up = Vector3::new(0.0, 0.0, 1.0);
    let right = forward.cross(&world_up).normalize();
    let up = right.cross(&forward);
    let down = -up;
    let rotation = Matrix3::from_row_slice(&[
        right.x, down.x, forward.x, //
        right.y, down.y, forward.y, //
        right.z, down.z, forward.z,
    ]);
    CameraExtrinsics::new(rotation, pos)
}

fn kb_params() -> KbParams {
    // The fisheye radius (240 px) must exceed the 40 px feather band, otherwise
    // every edge factor is 0.
    KbParams {
        width: 640,
        height: 480,
        fx: 300.0,
        fy: 300.0,
        cx: 320.0,
        cy: 240.0,
        ..Default::default()
    }
}

fn bowl_model() -> BowlModel {
    let params = BowlParams::new(2.0, 1.0, 0.5, 0.4, 4.0, 0.1, 2.2, 4.0, [0.0, 0.0]);
    let intrinsics = kb_params();
    let cameras: Vec<Camera> = [0.0f64, 90.0, 180.0, 270.0]
        .into_iter()
        .map(|az| {
            let model = make_camera_model("kb", &CameraModelParams::Kb(intrinsics.clone()))
                .expect("kb model");
            Camera::new(look_at_extrinsics(az, 4.0, 2.0, 0.5), model)
        })
        .collect();

    BowlModel::new(&params, &cameras)
}

fn bowl_scene(model: &BowlModel) -> SceneData {
    let images = (0..4)
        .map(|i| {
            image::RgbaImage::from_pixel(
                64,
                48,
                image::Rgba([(40 * (i + 1)) as u8, 20, 200, 255]),
            )
        })
        .collect();
    SceneData::from_bowl_model(model).with_camera_images(images)
}

/// Screen-covering triangle in the y-up render basis, seen from above.
fn covering_triangle() -> [PackedVertex; 3] {
    let mut vertices = [PackedVertex::default(); 3];
    vertices[0].position = [-10.0, 0.0, -10.0];
    vertices[1].position = [10.0, 0.0, -10.0];
    vertices[2].position = [0.0, 0.0, 10.0];
    vertices
}

fn triangle_scene(weight: [f32; 4], num_cameras: usize) -> SceneData {
    let mut vertices = covering_triangle();
    for vertex in &mut vertices {
        vertex.uv[0] = [0.5, 0.5, 0.0, 0.0];
        vertex.weights[0] = weight;
    }
    SceneData {
        vertices: vertices.to_vec(),
        indices: vec![0, 1, 2],
        num_cameras,
        camera_images: (0..num_cameras)
            .map(|_| image::RgbaImage::from_pixel(64, 48, image::Rgba([40, 20, 200, 255])))
            .collect(),
    }
}

/// A view whose near plane starts behind the bowl, so every vertex is clipped.
fn clipped_view() -> ViewSpec {
    ViewSpec::look_at(
        [0.0, 14.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        60.0,
        WIDTH as f32 / HEIGHT as f32,
        200.0,
        300.0,
    )
}

fn non_black_pixels(frame: &[u8]) -> usize {
    frame
        .chunks_exact(4)
        .filter(|pixel| pixel[0] != 0 || pixel[1] != 0 || pixel[2] != 0)
        .count()
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

/// Renders the views, or returns `None` when the machine has no adapter.
fn render_views(
    scene: &SceneData,
    views: &[ViewSpec],
    clear: [f64; 4],
) -> Option<Vec<Vec<u8>>> {
    render_views_at(scene, views, RenderSize::new(WIDTH, HEIGHT), clear)
}

/// Renders at an explicit size, or returns `None` when the machine has no
/// adapter.
fn render_views_at(
    scene: &SceneData,
    views: &[ViewSpec],
    size: RenderSize,
    clear: [f64; 4],
) -> Option<Vec<Vec<u8>>> {
    let _gpu = gpu_lock();
    match render_all_views_with(scene, views, size, clear) {
        Ok(frames) => Some(frames),
        Err(RenderError::NoAdapter) => {
            eprintln!("skipping offscreen check: no wgpu adapter available");
            None
        }
        Err(other) => panic!("unexpected render error: {other}"),
    }
}

fn render_single(scene: &SceneData) -> Option<Vec<u8>> {
    render_views(scene, &[top_down_view()], [0.0, 0.0, 0.0, 1.0])
        .map(|mut frames| frames.remove(0))
}

fn center_pixel(frame: &[u8]) -> [u8; 4] {
    pixel_at(frame, WIDTH / 2, HEIGHT / 2)
}

fn pixel_at(frame: &[u8], x: u32, y: u32) -> [u8; 4] {
    let offset = ((y * WIDTH + x) * 4) as usize;
    [
        frame[offset],
        frame[offset + 1],
        frame[offset + 2],
        frame[offset + 3],
    ]
}

fn is_black(pixel: [u8; 4]) -> bool {
    pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0
}

#[test]
fn bowl_cameras_bake_visible_vertices() {
    let model = bowl_model();
    let visible = (0..model.vertex_count())
        .filter(|i| model.baked_vertex(*i).weight.iter().any(|w| *w > 0.001))
        .count();
    assert!(
        visible > 0,
        "fixture cameras bake no weighted vertices: the render test would be vacuous"
    );
}

#[test]
fn renders_every_view_or_skips_without_adapter() {
    let model = bowl_model();
    let scene = bowl_scene(&model);
    // The second view is frustum-clipped empty, so a renderer that reuses the
    // first view's matrix for every frame cannot pass both assertions.
    let views = vec![top_down_view(), clipped_view()];

    if let Some(frames) = render_views(&scene, &views, [0.0, 0.0, 0.0, 1.0]) {
        {
            assert_eq!(views.len(), frames.len());
            for frame in &frames {
                assert_eq!((WIDTH * HEIGHT * 4) as usize, frame.len());
                // The shader always writes an opaque result, and the clear is opaque.
                assert!(frame.chunks_exact(4).all(|px| px[3] == 255));
            }
            assert!(
                non_black_pixels(&frames[0]) > 100,
                "the first view must show the bowl, got {} non-black pixels",
                non_black_pixels(&frames[0])
            );
            assert_eq!(
                0,
                non_black_pixels(&frames[1]),
                "the near=200 view clips the whole bowl and must stay black"
            );
        }
    }
}

#[test]
fn bowl_frame_has_non_black_pixels_in_the_centre_region() {
    let scene = bowl_scene(&bowl_model());
    let Some(frame) = render_single(&scene) else {
        return;
    };

    // The bowl (radius 4) seen from 14 m at 60 deg covers the central part of
    // the frame; the corners stay outside it.
    let mut centre_non_black = 0usize;
    for y in (HEIGHT / 4)..(3 * HEIGHT / 4) {
        for x in (WIDTH / 4)..(3 * WIDTH / 4) {
            if !is_black(pixel_at(&frame, x, y)) {
                centre_non_black += 1;
            }
        }
    }
    assert!(
        centre_non_black > 100,
        "expected the baked bowl in the centre region, got {centre_non_black} non-black pixels"
    );
    assert_eq!(
        [0, 0, 0, 255],
        pixel_at(&frame, 1, 1),
        "the top-left corner lies outside the bowl and must stay background"
    );
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
fn empty_scene_is_rejected() {
    let empty = SceneData {
        vertices: Vec::new(),
        indices: Vec::new(),
        num_cameras: 0,
        camera_images: Vec::new(),
    };
    let err = render_all_views(&empty, &[top_down_view()], RenderSize::new(WIDTH, HEIGHT))
        .expect_err("empty scene must fail");
    assert!(matches!(err, RenderError::EmptyScene));

    // Vertices without indices draw nothing; the guard must reject the pair.
    let vertices_only = SceneData {
        vertices: covering_triangle().to_vec(),
        indices: Vec::new(),
        num_cameras: 0,
        camera_images: Vec::new(),
    };
    let err = render_all_views(&vertices_only, &[top_down_view()], RenderSize::new(WIDTH, HEIGHT))
        .expect_err("vertices without indices must fail");
    assert!(matches!(err, RenderError::EmptyScene));
}

#[test]
fn more_cameras_than_the_packed_layout_is_rejected() {
    let mut scene = triangle_scene([1.0, 0.0, 0.0, 0.0], 1);
    scene.num_cameras = MAX_CAMERAS + 1;
    scene.camera_images = (0..=MAX_CAMERAS)
        .map(|_| image::RgbaImage::from_pixel(64, 48, image::Rgba([40, 20, 200, 255])))
        .collect();
    let err = render_all_views(&scene, &[top_down_view()], RenderSize::new(WIDTH, HEIGHT))
        .expect_err("9 cameras must be rejected");
    assert!(
        matches!(err, RenderError::TooManyCameras { .. }),
        "unexpected error: {err}"
    );
}

#[test]
fn camera_image_count_must_match_num_cameras() {
    let mut scene = triangle_scene([1.0, 0.0, 0.0, 0.0], 1);
    scene.num_cameras = 2;
    let err = render_all_views(&scene, &[top_down_view()], RenderSize::new(WIDTH, HEIGHT))
        .expect_err("missing camera layer must be rejected");
    assert!(
        matches!(err, RenderError::CameraImageCountMismatch { .. }),
        "unexpected error: {err}"
    );
}

/// Layer `layer` as a colour key: the red channel is a 7-texel-wide band
/// (`32 * x/7`), the green channel identifies the layer. Sampling a different
/// uv slot or a different array layer therefore changes the pixel.
fn gradient_layer(layer: usize) -> image::RgbaImage {
    image::RgbaImage::from_fn(64, 48, |x, _y| {
        image::Rgba([(32 * (x / 7)) as u8, (30 + 30 * layer) as u8, 0, 255])
    })
}

/// Camera `k` must read weight lane `k % 4` of slot `k / 4`, uv lane
/// `k % 2` of slot `k / 2`, and array layer `k`. Camera `k` samples texel
/// `x = 2 + 7k`, whose band is exactly `k`, so a crossed slot, lane or layer
/// changes the centre pixel (bands are 7 texels wide, i.e. robust to filtering).
#[test]
fn every_camera_slot_samples_its_own_layer_and_uv_slot() {
    const LAYERS: usize = 8;
    let centre_v = 24.5 / 48.0;

    let mut vertices = covering_triangle();
    for vertex in &mut vertices {
        for camera in 0..LAYERS {
            let u = (2.0 + 7.0 * camera as f32 + 0.5) / 64.0;
            let slot_uv = &mut vertex.uv[camera / 2];
            if camera % 2 == 0 {
                slot_uv[0] = u;
                slot_uv[1] = centre_v;
            } else {
                slot_uv[2] = u;
                slot_uv[3] = centre_v;
            }
        }
    }

    for camera in 0..LAYERS {
        let mut weighted = vertices;
        for vertex in &mut weighted {
            vertex.weights[camera / 4][camera % 4] = 1.0;
        }
        let scene = SceneData {
            vertices: weighted.to_vec(),
            indices: vec![0, 1, 2],
            num_cameras: LAYERS,
            camera_images: (0..LAYERS).map(gradient_layer).collect(),
        };
        let Some(frame) = render_single(&scene) else {
            return;
        };
        assert_eq!(
            [(32 * camera) as u8, (30 + 30 * camera) as u8, 0, 255],
            center_pixel(&frame),
            "camera {camera} sampled the wrong layer / uv slot / weight lane"
        );
    }
}

/// Maps world `x, y` in `[-1, 1]` straight to NDC and pins `z_ndc = 0.5`, so
/// screen-space tests need no camera.
fn screen_view() -> ViewSpec {
    ViewSpec::new([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.5, 1.0],
    ])
}

fn corner_scene() -> SceneData {
    let mut vertices = [PackedVertex::default(); 3];
    vertices[0].position = [-0.9, -0.9, 0.0];
    vertices[1].position = [-0.5, -0.9, 0.0];
    vertices[2].position = [-0.9, -0.5, 0.0];
    SceneData {
        vertices: vertices.to_vec(),
        indices: vec![0, 1, 2],
        num_cameras: 0,
        camera_images: Vec::new(),
    }
}

#[test]
fn clear_colour_fills_the_background() {
    let clear = [30.0 / 255.0, 36.0 / 255.0, 50.0 / 255.0, 1.0];
    let Some(frames) = render_views(&corner_scene(), &[screen_view()], clear) else {
        return;
    };
    assert_eq!([30, 36, 50, 255], center_pixel(&frames[0]));
}

/// The camera image's row 0 must reach the top of the frame, and its column 0
/// the left edge: a flipped or mirrored upload swaps the halves or the gradient.
#[test]
fn camera_image_row_zero_is_the_top_of_the_frame() {
    let image = image::RgbaImage::from_fn(64, 48, |x, y| {
        if y < 24 {
            image::Rgba([(x * 4) as u8, 0, 0, 255])
        } else {
            image::Rgba([0, 0, 255, 255])
        }
    });

    let mut vertices = [PackedVertex::default(); 4];
    let corners = [
        (-1.0, 1.0, 0.0, 0.0),
        (1.0, 1.0, 1.0, 0.0),
        (-1.0, -1.0, 0.0, 1.0),
        (1.0, -1.0, 1.0, 1.0),
    ];
    for (vertex, (x, y, u, v)) in vertices.iter_mut().zip(corners) {
        vertex.position = [x, y, 0.0];
        vertex.uv[0] = [u, v, 0.0, 0.0];
        vertex.weights[0] = [1.0, 0.0, 0.0, 0.0];
    }
    let scene = SceneData {
        vertices: vertices.to_vec(),
        indices: vec![0, 1, 2, 1, 3, 2],
        num_cameras: 1,
        camera_images: vec![image],
    };

    let Some(frames) = render_views(&scene, &[screen_view()], [0.0, 0.0, 0.0, 1.0]) else {
        return;
    };
    let frame = &frames[0];
    for x in [WIDTH / 8, WIDTH / 2, 7 * WIDTH / 8] {
        let top = pixel_at(frame, x, HEIGHT / 8);
        assert_eq!(
            [0, 0],
            [top[1], top[2]],
            "x = {x}: image rows 0..23 (red gradient) must paint the top half, got {top:?}"
        );
        assert_eq!(
            [0, 0, 255, 255],
            pixel_at(frame, x, 7 * HEIGHT / 8),
            "x = {x}: image rows 24..47 must paint the bottom half"
        );
    }

    let left = pixel_at(frame, WIDTH / 8, HEIGHT / 8)[0];
    let right = pixel_at(frame, 7 * WIDTH / 8, HEIGHT / 8)[0];
    assert!(
        left < 60 && right > 180,
        "column 0 must be on the left: red = {left} then {right}"
    );
}

const SHIP_WIDTH: u32 = 960;
const SHIP_HEIGHT: u32 = 640;
const SHIP_CAMERA_COUNT: usize = 8;

fn ship_bowl() -> BowlParams {
    BowlParams::for_car_size(211.6, 50.0, 50.0)
}

fn ship_cameras() -> Vec<Camera> {
    let intrinsics = kb_params();
    (0..SHIP_CAMERA_COUNT)
        .map(|index| {
            let model = make_camera_model("kb", &CameraModelParams::Kb(intrinsics.clone()))
                .expect("kb model");
            let azimuth = index as f64 * 45.0;
            Camera::new(look_at_extrinsics(azimuth, 220.0, 80.0, 0.0), model)
        })
        .collect()
}

fn ship_scene(bowl: &BowlParams) -> SceneData {
    const COLOURS: [[u8; 3]; 4] = [
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 0],
    ];
    let model = BowlModel::new(bowl, &ship_cameras());
    let images = (0..SHIP_CAMERA_COUNT)
        .map(|index| {
            let colour = COLOURS[index % 4];
            image::RgbaImage::from_pixel(
                640,
                480,
                image::Rgba([colour[0], colour[1], colour[2], 255]),
            )
        })
        .collect();
    SceneData::from_bowl_model(&model).with_camera_images(images)
}

fn to_render_basis(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[2] as f32, -p[1] as f32]
}

/// The four default views. `clip_bowl` supplies the clip planes; this case only
/// asserts the frame is not all background, so clip-plane scaling is covered
/// separately by `clip_planes_ship_scale_cover_the_bowl`.
fn default_view_specs(pose_bowl: &BowlParams, clip_bowl: &BowlParams) -> Vec<ViewSpec> {
    let clip_poses = clip_bowl.default_view_poses();
    let max_eye = clip_poses.iter().fold(0.0f64, |max, pose| {
        let d = [
            pose.position[0] - pose.target[0],
            pose.position[1] - pose.target[1],
            pose.position[2] - pose.target[2],
        ];
        max.max((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt())
    });
    let clip = clip_bowl.clip_planes_for(max_eye);
    pose_bowl
        .default_view_poses()
        .iter()
        .map(|pose| {
            ViewSpec::look_at(
                to_render_basis(pose.position),
                to_render_basis(pose.target),
                to_render_basis(pose.up),
                pose.fovy_deg as f32,
                SHIP_WIDTH as f32 / SHIP_HEIGHT as f32,
                clip.near as f32,
                clip.far as f32,
            )
        })
        .collect()
}

fn vivid_pixels(frame: &[u8]) -> usize {
    frame
        .chunks_exact(4)
        .filter(|pixel| {
            let min = pixel[0].min(pixel[1]).min(pixel[2]);
            let max = pixel[0].max(pixel[1]).max(pixel[2]);
            max - min > 100
        })
        .count()
}

#[test]
fn ship_scale_views_are_not_background() {
    let ship = ship_bowl();
    assert!(ship.outer_radius() > 450.0, "the fixture must be ship scale");
    let scene = ship_scene(&ship);
    let views = default_view_specs(&ship, &ship);
    let Some(frames) = render_views_at(
        &scene,
        &views,
        RenderSize::new(SHIP_WIDTH, SHIP_HEIGHT),
        [0.0, 0.0, 0.0, 1.0],
    ) else {
        return;
    };
    assert_eq!(4, frames.len());
    for (index, frame) in frames.iter().enumerate() {
        let vivid = vivid_pixels(frame);
        assert!(
            vivid > 1000,
            "ship view {index} has only {vivid} vivid pixels"
        );
    }
}
