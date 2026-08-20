#include "core/calib_files_generator.hpp"
#include "util/camera_name.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace avm_worker { namespace avm_stitch {

namespace fs = std::filesystem;

// ── Internal helpers (mirror avm-worker stitch_eval.cpp exactly) ──

namespace {

// Format double as scientific with full precision — must stay identical to
// stitch_eval.cpp's fmt_double (model_creator depends on the file format).
std::string fmt_double(double val) {
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(16) << val;
    return oss.str();
}

// Read a template file and replace every {{KEY}} placeholder with vars[key].
// Must stay identical to stitch_eval.cpp's render_template.
std::string render_template(const std::string& tmpl_path,
                            const std::map<std::string, std::string>& vars) {
    std::ifstream f(tmpl_path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open template: " + tmpl_path);
    }
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());

    for (const auto& [key, val] : vars) {
        std::string placeholder = "{{" + key + "}}";
        size_t pos = 0;
        while ((pos = content.find(placeholder, pos)) != std::string::npos) {
            content.replace(pos, placeholder.length(), val);
            pos += val.length();
        }
    }
    return content;
}

// Fail fast when any {{...}} placeholder survived rendering: model_creator
// parses unreplaced literals as DBL_MAX and still exits 0, silently
// producing black frames (production incident). Throwing here — naming the
// output file and the leftover placeholders (camera role info) — turns the
// mismatch into a visible stitch_error instead.
void EnsureNoUnreplacedPlaceholders(const std::string& content,
                                    const std::string& out_path) {
    size_t pos = content.find("{{");
    if (pos == std::string::npos) return;

    std::string leftovers;
    for (int shown = 0; pos != std::string::npos && shown < 4; ++shown) {
        size_t end = content.find("}}", pos);
        size_t len = (end == std::string::npos) ? std::string::npos
                                                : end + 2 - pos;
        leftovers += (shown ? ", " : "") + content.substr(pos, len);
        pos = (end == std::string::npos) ? std::string::npos : end + 2;
    }
    throw std::runtime_error(
        "Unreplaced template placeholders [" + leftovers +
        "] in generated calib XML: " + out_path +
        " (camera labels do not match the template's camera roles)");
}

// Iterates calib["cameras"], resolves label-style tags (Front/Rear/Left/Right
// matching {{FX_Front}} style placeholders), fills `vars` via the per-field
// populator, then renders the template and writes the output. Mirrors
// stitch_eval.cpp's GenCalibXml.
using CalibFieldPopulator = void (*)(std::map<std::string, std::string>& vars,
                                      const std::string& tag,
                                      const nlohmann::json& cam);

void GenCalibXml(const nlohmann::json& calib,
                 const std::string& tmpl_path,
                 const std::string& out_path,
                 CalibFieldPopulator populate) {
    std::map<std::string, std::string> vars;
    for (size_t i = 0; i < calib["cameras"].size(); ++i) {
        const auto& cam = calib["cameras"][i];
        std::string tag = ResolveCameraLabel(cam, i);
        populate(vars, tag, cam);
    }

    std::string xml = render_template(tmpl_path, vars);
    EnsureNoUnreplacedPlaceholders(xml, out_path);  // before any disk write
    std::ofstream f(out_path);
    if (!f.is_open()) throw std::runtime_error("Cannot write: " + out_path);
    f << xml;
}

} // anonymous namespace

// ── Public API ──

CalibFilesGenerator::CalibFilesGenerator(const std::string& templates_dir)
    : templates_dir_(templates_dir) {}

void CalibFilesGenerator::Generate(const nlohmann::json& calibration,
                                   const std::string& output_dir) {
    // Validate calibration up-front: model_creator requires a non-empty
    // cameras array (a missing/empty array yields no usable intrinsic data).
    if (!calibration.is_object() || !calibration.contains("cameras") ||
        !calibration["cameras"].is_array() || calibration["cameras"].empty()) {
        throw std::runtime_error("Empty cameras array in calibration");
    }

    fs::create_directories(output_dir + "/calib");
    fs::create_directories(output_dir + "/resources/config");

    // ── intrinsics_params.xml ──
    GenCalibXml(calibration,
                templates_dir_ + "/calib/intrinsics_params.xml.template",
                output_dir + "/calib/intrinsics_params.xml",
        [](std::map<std::string, std::string>& vars, const std::string& tag,
           const nlohmann::json& cam) {
            vars["FX_" + tag] = fmt_double(cam["fx"].get<double>());
            vars["FY_" + tag] = fmt_double(cam["fy"].get<double>());
            vars["CX_" + tag] = fmt_double(cam["cx"].get<double>());
            vars["CY_" + tag] = fmt_double(cam["cy"].get<double>());
            vars["K1_" + tag] = fmt_double(cam["k1"].get<double>());
            vars["K2_" + tag] = fmt_double(cam["k2"].get<double>());
            vars["K3_" + tag] = fmt_double(cam["k3"].get<double>());
            vars["K4_" + tag] = fmt_double(cam["k4"].get<double>());
        });

    // ── extrinsics_params.xml ──
    GenCalibXml(calibration,
                templates_dir_ + "/calib/extrinsics_params.xml.template",
                output_dir + "/calib/extrinsics_params.xml",
        [](std::map<std::string, std::string>& vars, const std::string& tag,
           const nlohmann::json& cam) {
            auto r = cam["rotation"];
            auto t = cam["translation"];
            double r00 = r[0].get<double>(), r01 = r[1].get<double>(), r02 = r[2].get<double>();
            double r10 = r[3].get<double>(), r11 = r[4].get<double>(), r12 = r[5].get<double>();
            double r20 = r[6].get<double>(), r21 = r[7].get<double>(), r22 = r[8].get<double>();
            double tx = t[0].get<double>(), ty = t[1].get<double>(), tz = t[2].get<double>();

            // T_wc (camera->world) rotation, row-major
            vars["ROT_" + tag + "_00"] = fmt_double(r00);
            vars["ROT_" + tag + "_01"] = fmt_double(r01);
            vars["ROT_" + tag + "_02"] = fmt_double(r02);
            vars["ROT_" + tag + "_10"] = fmt_double(r10);
            vars["ROT_" + tag + "_11"] = fmt_double(r11);
            vars["ROT_" + tag + "_12"] = fmt_double(r12);
            vars["ROT_" + tag + "_20"] = fmt_double(r20);
            vars["ROT_" + tag + "_21"] = fmt_double(r21);
            vars["ROT_" + tag + "_22"] = fmt_double(r22);

            // T_wc (camera->world) translation
            vars["T_" + tag + "_X"] = fmt_double(tx);
            vars["T_" + tag + "_Y"] = fmt_double(ty);
            vars["T_" + tag + "_Z"] = fmt_double(tz);
        });

    // ── models_config.json ──
    const auto& cams = calibration["cameras"];
    const auto& car = calibration["car"];
    const auto& cam0 = cams[0];

    std::map<std::string, std::string> vars;
    vars["CAR_LENGTH"]    = fmt_double(car["length"].get<double>());
    vars["CAR_WIDTH"]     = fmt_double(car["width"].get<double>());
    vars["CAR_AXIS_X"]    = fmt_double(car["axis_x"].get<double>());
    vars["CAR_AXIS_Y"]    = fmt_double(car["axis_y"].get<double>());
    vars["CAMERA_WIDTH"]  = std::to_string(cam0["width"].get<int>());
    vars["CAMERA_HEIGHT"] = std::to_string(cam0["height"].get<int>());

    std::string json = render_template(
        templates_dir_ + "/resources/config/models_config.json.template", vars);
    std::ofstream f(output_dir + "/resources/config/models_config.json");
    if (!f.is_open()) {
        throw std::runtime_error("Cannot write: " +
            output_dir + "/resources/config/models_config.json");
    }
    f << json;
}

}} // namespace avm_worker::avm_stitch
