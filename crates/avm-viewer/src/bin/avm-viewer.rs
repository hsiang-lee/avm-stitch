use std::path::PathBuf;
use std::process::ExitCode;

use avm_core::bowl::BowlModel;
use avm_core::calibration::Calibration;
use avm_viewer::ViewerOptions;

const DEFAULT_DATA_DIR: &str = "examples/data_8";

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(args) => args,
        Err(error) => {
            eprintln!("avm-viewer: {error}");
            return ExitCode::FAILURE;
        }
    };
    match run(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("avm-viewer: {error}");
            ExitCode::FAILURE
        }
    }
}

struct Args {
    data_dir: PathBuf,
    options: ViewerOptions,
}

fn parse_args() -> Result<Args, String> {
    let mut data_dir = None;
    let mut frames = None;
    let mut screenshot = None;
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--frames" => {
                let value = arguments.next().ok_or("--frames needs a frame count")?;
                let count: u32 = value
                    .parse()
                    .map_err(|_| format!("--frames expects a positive integer, got {value}"))?;
                if count == 0 {
                    return Err("--frames must be at least 1".into());
                }
                frames = Some(count);
            }
            "--screenshot" => {
                let value = arguments.next().ok_or("--screenshot needs a path")?;
                screenshot = Some(PathBuf::from(value));
            }
            option if option.starts_with("--") => {
                return Err(format!("unknown option {option}"));
            }
            directory if data_dir.is_none() => data_dir = Some(PathBuf::from(directory)),
            extra => return Err(format!("unexpected argument {extra}")),
        }
    }
    Ok(Args {
        data_dir: data_dir.unwrap_or_else(|| PathBuf::from(DEFAULT_DATA_DIR)),
        options: ViewerOptions {
            frames,
            screenshot,
        },
    })
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let json_path = args.data_dir.join("calibration.json");
    let calibration = Calibration::load_from_path(&json_path)?;
    let model = BowlModel::new(&calibration.bowl_params(), &calibration.cameras);
    println!("Controls:  LMB=rotate  RMB=zoom  MMB=pan");
    avm_viewer::run_windowed(&args.data_dir, &calibration, &model, &args.options)?;
    Ok(())
}
