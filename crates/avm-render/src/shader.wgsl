// WGSL port of the GLSL 330 shader pair in src/render/stitch_scene.cpp.
//
// Behaviour-preserving differences from the GLSL original:
// * `uNumCameras` is read straight from the uniform buffer in the fragment
//   stage instead of being carried by a `flat out` varying.
// * `sampler2DArray` becomes `texture_2d_array` plus an explicit sampler.

struct Uniforms {
    view_proj: mat4x4<f32>,
    num_cameras: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

@group(1) @binding(0) var camera_textures: texture_2d_array<f32>;
@group(1) @binding(1) var camera_sampler: sampler;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv_coord0: vec4<f32>,
    @location(3) uv_coord1: vec4<f32>,
    @location(4) uv_coord2: vec4<f32>,
    @location(5) uv_coord3: vec4<f32>,
    @location(6) weights0: vec4<f32>,
    @location(7) weights1: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) frag_normal: vec3<f32>,
    @location(1) frag_uv0: vec4<f32>,
    @location(2) frag_uv1: vec4<f32>,
    @location(3) frag_uv2: vec4<f32>,
    @location(4) frag_uv3: vec4<f32>,
    @location(5) frag_weights0: vec4<f32>,
    @location(6) frag_weights1: vec4<f32>,
};

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var output: VertexOutput;
    output.clip_position = uniforms.view_proj * vec4<f32>(input.position, 1.0);
    output.frag_normal = input.normal;
    output.frag_uv0 = input.uv_coord0;
    output.frag_uv1 = input.uv_coord1;
    output.frag_uv2 = input.uv_coord2;
    output.frag_uv3 = input.uv_coord3;
    output.frag_weights0 = input.weights0;
    output.frag_weights1 = input.weights1;
    return output;
}

const WEIGHT_MIN: f32 = 0.001;

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    var result = vec3<f32>(0.0, 0.0, 0.0);

    // Camera 0
    if (uniforms.num_cameras > 0u && input.frag_weights0.x > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv0.xy, 0).rgb * input.frag_weights0.x;
    }
    // Camera 1
    if (uniforms.num_cameras > 1u && input.frag_weights0.y > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv0.zw, 1).rgb * input.frag_weights0.y;
    }
    // Camera 2
    if (uniforms.num_cameras > 2u && input.frag_weights0.z > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv1.xy, 2).rgb * input.frag_weights0.z;
    }
    // Camera 3
    if (uniforms.num_cameras > 3u && input.frag_weights0.w > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv1.zw, 3).rgb * input.frag_weights0.w;
    }
    // Camera 4
    if (uniforms.num_cameras > 4u && input.frag_weights1.x > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv2.xy, 4).rgb * input.frag_weights1.x;
    }
    // Camera 5
    if (uniforms.num_cameras > 5u && input.frag_weights1.y > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv2.zw, 5).rgb * input.frag_weights1.y;
    }
    // Camera 6
    if (uniforms.num_cameras > 6u && input.frag_weights1.z > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv3.xy, 6).rgb * input.frag_weights1.z;
    }
    // Camera 7
    if (uniforms.num_cameras > 7u && input.frag_weights1.w > WEIGHT_MIN) {
        result += textureSample(camera_textures, camera_sampler, input.frag_uv3.zw, 7).rgb * input.frag_weights1.w;
    }

    return vec4<f32>(result, 1.0);
}
