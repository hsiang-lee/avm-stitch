//! Vertex packing contract: identical attribute layout to `BuildBowlGeometry`.

use avm_core::bowl::{BakedVertex, BowlModel, BowlParams};
use avm_render::vertex::{
    PackedVertex, VertexAttributes, NUM_UV_SLOTS, NUM_WEIGHT_SLOTS, PACKED_VERTEX_SIZE,
};
use avm_render::SceneData;

fn baked_with(num_cameras: usize, seed: f32) -> BakedVertex {
    let mut baked = BakedVertex::new(num_cameras);
    for c in 0..num_cameras {
        let base = seed + c as f32;
        baked.uv[c] = [base + 0.25, base + 0.5];
        baked.weight[c] = base;
    }
    baked
}

#[test]
fn packed_vertex_size_matches_the_cpp_packing() {
    assert_eq!(120, PACKED_VERTEX_SIZE);
    assert_eq!(PACKED_VERTEX_SIZE, std::mem::size_of::<PackedVertex>());
    assert_eq!(4, std::mem::align_of::<PackedVertex>());
}

#[test]
fn vertex_attributes_use_the_cpp_locations_and_offsets() {
    let attrs = &VertexAttributes::ATTRS;
    assert_eq!(8, attrs.len());
    let expected_locations = [0u32, 1, 2, 3, 4, 5, 6, 7];
    let expected_offsets = [0u64, 12, 24, 40, 56, 72, 88, 104];
    for (i, attr) in attrs.iter().enumerate() {
        assert_eq!(expected_locations[i], attr.shader_location, "location {i}");
        assert_eq!(expected_offsets[i], attr.offset, "offset {i}");
    }
    assert_eq!(
        wgpu::VertexFormat::Float32x3,
        attrs[0].format,
        "position is vec3"
    );
    assert_eq!(
        wgpu::VertexFormat::Float32x3,
        attrs[1].format,
        "normal is vec3"
    );
    for attr in &attrs[2..] {
        assert_eq!(wgpu::VertexFormat::Float32x4, attr.format);
    }
    assert_eq!(
        PACKED_VERTEX_SIZE as u64,
        PackedVertex::layout().array_stride
    );
    assert_eq!(8, PackedVertex::layout().attributes.len());
}

#[test]
fn from_baked_packs_two_cameras_per_uv_slot_and_four_per_weight_slot() {
    let baked = baked_with(5, 10.0);
    let packed = PackedVertex::from_baked(&[0.0; 3], &[0.0; 3], &baked);

    // Camera 0 / 1 -> uv slot 0
    assert_eq!([10.25, 10.5, 11.25, 11.5], packed.uv[0]);
    // Camera 2 / 3 -> uv slot 1
    assert_eq!([12.25, 12.5, 13.25, 13.5], packed.uv[1]);
    // Camera 4 -> first half of uv slot 2, second half zero
    assert_eq!([14.25, 14.5, 0.0, 0.0], packed.uv[2]);
    assert_eq!([0.0; 4], packed.uv[3]);

    assert_eq!([10.0, 11.0, 12.0, 13.0], packed.weights[0]);
    assert_eq!([14.0, 0.0, 0.0, 0.0], packed.weights[1]);
}

#[test]
fn from_baked_zero_fills_cameras_beyond_eight() {
    let baked = baked_with(9, 1.0);
    let packed = PackedVertex::from_baked(&[0.0; 3], &[0.0; 3], &baked);

    // Camera 8 (value 9.0) cannot be represented: instead of leaking it we
    // keep the slot zero, exactly like the C++ packer (8-way limit).
    assert!(!packed.uv.iter().flatten().any(|v| *v == 9.25));
    assert!(!packed.weights.iter().flatten().any(|v| *v == 9.0));
    assert_eq!(NUM_UV_SLOTS * 2, 8);
    assert_eq!(NUM_WEIGHT_SLOTS * 4, 8);
}

#[test]
fn position_and_normal_use_the_osg_render_basis() {
    let packed = PackedVertex::from_baked(&[1.0, 2.0, 3.0], &[4.0, 5.0, 6.0], &baked_with(0, 0.0));
    assert_eq!([1.0, 3.0, -2.0], packed.position);
    assert_eq!([4.0, 6.0, -5.0], packed.normal);
}

#[test]
fn scene_data_packs_the_bowl_mesh_and_index_buffer() {
    let params = BowlParams::new(2.0, 1.0, 0.5, 0.4, 4.0, 0.1, 2.2, 4.0, [0.0, 0.0]);
    let intrinsics = avm_core::camera::KbParams {
        width: 640,
        height: 480,
        fx: 300.0,
        fy: 300.0,
        cx: 320.0,
        cy: 240.0,
        ..Default::default()
    };
    let extrinsics = avm_core::camera::CameraExtrinsics::identity();
    let cameras: Vec<_> = (0..2)
        .map(|_| {
            let model = avm_core::camera::make_camera_model(
                "kb",
                &avm_core::camera::CameraModelParams::Kb(intrinsics.clone()),
            )
            .expect("kb model");
            avm_core::camera::Camera::new(extrinsics, model)
        })
        .collect();
    let model = BowlModel::new(&params, &cameras);

    let scene = SceneData::from_bowl_model(&model);
    assert_eq!(model.vertex_count(), scene.vertices.len());
    assert_eq!(model.triangle_count() * 3, scene.indices.len());
    assert_eq!(2, scene.num_cameras);

    for (i, tri) in (0..model.triangle_count()).map(|t| model.triangle(t)).enumerate() {
        assert_eq!(tri[0] as u32, scene.indices[i * 3]);
        assert_eq!(tri[1] as u32, scene.indices[i * 3 + 1]);
        assert_eq!(tri[2] as u32, scene.indices[i * 3 + 2]);
    }

    // Positions go through the same basis conversion as the C++ builder.
    let v = model.vertex(0);
    assert_eq!(v[0] as f32, scene.vertices[0].position[0]);
    assert_eq!(v[2] as f32, scene.vertices[0].position[1]);
    assert_eq!(-v[1] as f32, scene.vertices[0].position[2]);
}
