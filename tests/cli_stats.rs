use std::process::Command;

fn cli() -> Command {
    Command::new(env!("CARGO_BIN_EXE_avm-cli"))
}

fn examples_data(set: &str) -> String {
    format!("{}/examples/{set}", env!("CARGO_MANIFEST_DIR"))
}

fn line_containing<'a>(stdout: &'a str, needle: &str) -> &'a str {
    stdout
        .lines()
        .find(|line| line.contains(needle))
        .unwrap_or_else(|| panic!("stdout has no line containing {needle:?}:\n{stdout}"))
}

fn slice_between<'a>(text: &'a str, prefix: &str, suffix: &str) -> &'a str {
    let rest = text
        .split_once(prefix)
        .unwrap_or_else(|| panic!("missing {prefix:?} in {text:?}"))
        .1;
    rest.split_once(suffix)
        .map(|(value, _)| value)
        .unwrap_or_else(|| panic!("missing {suffix:?} after {prefix:?} in {text:?}"))
}

fn parse_number<T: std::str::FromStr>(text: &str, what: &str) -> T {
    text.trim()
        .parse()
        .unwrap_or_else(|_| panic!("{what} is not a number: {text:?}"))
}

fn assert_statistics(stdout: &str, expected_cameras: usize) {
    let loaded = line_containing(stdout, "Loaded ");
    let cameras: usize = parse_number(
        slice_between(loaded, "Loaded ", " cameras"),
        "camera count",
    );
    assert_eq!(expected_cameras, cameras, "stdout:\n{stdout}");

    let bowl = line_containing(stdout, "Bowl model:");
    let vertices: usize = parse_number(
        slice_between(bowl, "Bowl model: ", " vertices"),
        "vertex count",
    );
    let triangles: usize = parse_number(
        slice_between(bowl, " vertices, ", " triangles"),
        "triangle count",
    );
    assert!(vertices > 0, "stdout:\n{stdout}");
    assert!(triangles > 0, "stdout:\n{stdout}");

    let weights = line_containing(stdout, "Weight sums:");
    let min_sum: f64 = parse_number(
        slice_between(weights, "Weight sums: ", ".."),
        "min weight sum",
    );
    let max_sum: f64 = parse_number(
        slice_between(weights, "..", " (visible vertices: "),
        "max weight sum",
    );
    let visible: usize = parse_number(
        slice_between(weights, "(visible vertices: ", "/"),
        "visible vertex count",
    );
    let total: usize = parse_number(
        slice_between(weights, "/", ")"),
        "total vertex count",
    );

    assert!(0.0 <= min_sum, "min weight sum {min_sum}\nstdout:\n{stdout}");
    assert!(
        min_sum <= max_sum,
        "min {min_sum} > max {max_sum}\nstdout:\n{stdout}"
    );
    assert!(max_sum <= 1.0 + 1e-6, "max weight sum {max_sum}");
    assert!(visible > 0, "stdout:\n{stdout}");
    assert!(visible <= total, "visible {visible} > total {total}");
    assert_eq!(vertices, total, "weight-sum total must be the vertex count");
}

#[test]
fn data_4_reports_camera_mesh_and_weight_statistics() {
    let output = cli()
        .arg(examples_data("data_4"))
        .output()
        .expect("avm-cli must run");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(output.status.success(), "stdout:\n{stdout}");
    assert_statistics(&stdout, 4);
    assert!(stdout.contains("front"), "camera names missing:\n{stdout}");
}

#[test]
fn data_8_reports_eight_cameras() {
    let output = cli()
        .arg(examples_data("data_8"))
        .output()
        .expect("avm-cli must run");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(output.status.success(), "stdout:\n{stdout}");
    assert_statistics(&stdout, 8);
    assert!(stdout.contains("cam_7"), "stdout:\n{stdout}");
}

#[test]
fn missing_calibration_fails_with_a_message() {
    let output = cli()
        .arg(examples_data("no_such_data_set"))
        .output()
        .expect("avm-cli must run");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(!output.status.success());
    assert!(
        stderr.contains("calibration.json"),
        "stderr should name the missing file, got:\n{stderr}"
    );
}
