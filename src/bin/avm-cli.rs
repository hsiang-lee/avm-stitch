use std::path::Path;
use std::process::ExitCode;

use avm_stitch::bowl::BowlModel;
use avm_stitch::calibration::Calibration;

const DEFAULT_DATA_DIR: &str = "examples/data_8";

fn main() -> ExitCode {
    let mut data_dir = None;
    for argument in std::env::args().skip(1) {
        if argument.starts_with("--") {
            eprintln!(
                "avm-cli: unknown option {argument} (the interactive viewer is \
                 `cargo run --example avm_viewer -- <data_dir>`)"
            );
            return ExitCode::FAILURE;
        }
        if data_dir.is_some() {
            eprintln!("avm-cli: unexpected argument {argument}");
            return ExitCode::FAILURE;
        }
        data_dir = Some(argument);
    }
    let data_dir = data_dir.unwrap_or_else(|| DEFAULT_DATA_DIR.to_string());
    match run(&data_dir) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("avm-cli: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(data_dir: &str) -> Result<(), Box<dyn std::error::Error>> {
    let json_path = Path::new(data_dir).join("calibration.json");
    let calibration = Calibration::load_from_path(&json_path)?;

    println!(
        "Loaded {} cameras from {}",
        calibration.cameras.len(),
        json_path.display()
    );
    for (camera, image) in calibration.cameras.iter().zip(&calibration.image_files) {
        println!(
            "  {}: {} ({}x{})",
            camera.name(),
            if image.is_empty() { "<no image>" } else { image },
            camera.width(),
            camera.height()
        );
    }

    let bowl = calibration.bowl_params();
    let car_center = bowl.car_center();
    println!(
        "Car origin: {} (axis_x={} -> car_center=({:.4}, {:.4}))",
        calibration.car.origin_convention, calibration.car.axis_x, car_center[0], car_center[1]
    );
    println!(
        "Derived bowl params: OuterRadius={:.4} FlatPlatformRadius={:.4} GridStep={:.4} \
         SlopeCoefficient={:.6} HeightExponent={:.2} MaxHeight={:.4}",
        bowl.outer_radius(),
        bowl.flat_platform_radius(),
        bowl.grid_step(),
        bowl.slope_coefficient(),
        bowl.height_exponent(),
        bowl.max_height()
    );

    let model = BowlModel::new(&bowl, &calibration.cameras);
    println!(
        "Bowl model: {} vertices, {} triangles",
        model.vertex_count(),
        model.triangle_count()
    );

    let (min_sum, max_sum, visible) = weight_sum_range(&model);
    println!(
        "Weight sums: {min_sum:.6}..{max_sum:.6} (visible vertices: {visible}/{})",
        model.vertex_count()
    );

    Ok(())
}

fn weight_sum_range(model: &BowlModel) -> (f64, f64, usize) {
    if model.vertex_count() == 0 {
        return (0.0, 0.0, 0);
    }
    let mut min_sum = f64::INFINITY;
    let mut max_sum = f64::NEG_INFINITY;
    let mut visible = 0;
    for i in 0..model.vertex_count() {
        let sum: f64 = model
            .baked_vertex(i)
            .weight
            .iter()
            .copied()
            .map(f64::from)
            .sum();
        if sum > 1e-9 {
            visible += 1;
        }
        min_sum = min_sum.min(sum);
        max_sum = max_sum.max(sum);
    }
    (min_sum, max_sum, visible)
}
