//! The shipped WGSL must parse and pass naga validation (AC A7).

use avm_render::SHADER_SOURCE;

fn parse() -> naga::Module {
    naga::front::wgsl::parse_str(SHADER_SOURCE).expect("WGSL must parse")
}

/// The value type behind a global variable, unwrapping its pointer if any.
fn pointee<'a>(module: &'a naga::Module, ty: naga::Handle<naga::Type>) -> &'a naga::TypeInner {
    match &module.types[ty].inner {
        naga::TypeInner::Pointer { base, .. } => &module.types[*base].inner,
        inner => inner,
    }
}

#[test]
fn shader_parses_and_validates_with_naga() {
    let module = parse();
    let mut validator = naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::all(),
    );
    validator.validate(&module).expect("WGSL must validate");
}

#[test]
fn shader_uses_a_texture_array_and_the_hard_weight_gate() {
    let module = parse();

    // The camera texture is bound as an array of 2D images.
    let has_arrayed_d2_image = module.global_variables.iter().any(|(_, global)| {
        matches!(
            pointee(&module, global.ty),
            naga::TypeInner::Image {
                dim: naga::ImageDimension::D2,
                arrayed: true,
                ..
            }
        )
    });
    assert!(
        has_arrayed_d2_image,
        "no global holds a texture_2d_array-like image type"
    );

    // One guarded sample per camera layer.
    let fragment = module
        .entry_points
        .iter()
        .find(|entry| entry.name == "fs_main")
        .expect("fs_main entry point");
    let samples = fragment
        .function
        .expressions
        .iter()
        .filter(|(_, expr)| matches!(expr, naga::Expression::ImageSample { .. }))
        .count();
    assert_eq!(8, samples, "all eight camera layers must be sampled");

    // The hard weight gate uses the constant 0.001.
    let has_weight_min = module.global_expressions.iter().any(|(_, expr)| {
        matches!(expr, naga::Expression::Literal(naga::Literal::F32(value)) if *value == 0.001)
    });
    assert!(
        has_weight_min,
        "the 0.001 hard-gate literal must be a constant expression"
    );
}

#[test]
fn shader_has_the_expected_entry_points() {
    let module = parse();
    let vertex = module
        .entry_points
        .iter()
        .find(|entry| entry.stage == naga::ShaderStage::Vertex)
        .expect("vertex entry point");
    let fragment = module
        .entry_points
        .iter()
        .find(|entry| entry.stage == naga::ShaderStage::Fragment)
        .expect("fragment entry point");
    assert_eq!("vs_main", vertex.name);
    assert_eq!("fs_main", fragment.name);
}
