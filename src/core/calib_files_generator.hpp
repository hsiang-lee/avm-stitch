#pragma once

#include <nlohmann/json_fwd.hpp>

#include <string>

namespace avm_worker { namespace avm_stitch {

/// Generates the calibration XML/JSON input files required by model_creator
/// before it can produce .bin model files:
///   <output_dir>/calib/intrinsics_params.xml
///   <output_dir>/calib/extrinsics_params.xml
///   <output_dir>/resources/config/models_config.json
///
/// This component is the avm-stitch equivalent of the GenCalibFiles /
/// GenCalibXml / gen_intrinsics_xml / gen_extrinsics_xml / gen_models_config
/// helpers in avm-worker's stitch_eval.cpp; the template rendering logic is
/// kept identical so model_creator sees the same file format.
///
/// Usage:
///   CalibFilesGenerator gen("/path/to/templates");  // contains calib/*.template
///   gen.Generate(task["calibration"], "/tmp/stitch/task-1");
class CalibFilesGenerator {
public:
    /// @param templates_dir  directory containing calib/*.xml.template and
    ///                       resources/config/*.json.template files
    explicit CalibFilesGenerator(const std::string& templates_dir);

    /// Generate the three calibration files under `output_dir` (the calib/
    /// and resources/config/ subdirectories are created automatically).
    /// @param calibration  task["calibration"] JSON subtree (must contain a
    ///                     non-empty "cameras" array and a "car" object)
    /// @param output_dir   root output directory
    /// @throws std::runtime_error if calibration is invalid or a template
    ///                            file cannot be opened
    void Generate(const nlohmann::json& calibration,
                  const std::string& output_dir);

private:
    std::string templates_dir_;
};

}} // namespace avm_worker::avm_stitch
