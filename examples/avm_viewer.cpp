#include "bowl_model/bowl_model.hpp"
#include "bowl_model/bowl_params.hpp"
#include "camera_model/camera.hpp"
#include "camera_model/camera_model.hpp"
#include "render/osg_view_renderer.hpp"
#include "render/stitch_scene.hpp"

#include <nlohmann/json.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace avm_stitch;

namespace {

struct ViewerConfig {
    std::vector<Camera> cameras;
    std::vector<std::string> camera_names;
    std::vector<cv::Mat> images;
    std::string car_origin_convention;
    double axis_x = 0.0;
    BowlParams bowl;
};

ViewerConfig LoadViewerConfig(const std::string& json_path, const std::string& image_dir) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open " << json_path << "\n";
        return {};
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();
    if (content.empty() || content.back() != '\n')
        content += '\n';

    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(content);

        ViewerConfig cfg;
        for (std::size_t i = 0; i < doc["cameras"].size(); ++i) {
            const auto& node = doc["cameras"][i];
            const std::string name = node.value("name", "");
            const std::string image = node.value("image", "");

            const auto& intr = node["intrinsics"];
            KbParams kb{node.value("width", 0),
                        node.value("height", 0),
                        intr.value("fx", 0.0),
                        intr.value("fy", 0.0),
                        intr.value("cx", 0.0),
                        intr.value("cy", 0.0),
                        intr.value("k1", 0.0),
                        intr.value("k2", 0.0),
                        intr.value("k3", 0.0),
                        intr.value("k4", 0.0)};

            cv::Matx33d R = cv::Matx33d::eye();
            cv::Vec3d t = cv::Vec3d::zeros();
            const auto& r = node["extrinsics"]["r"];
            for (std::size_t row = 0; row < 3 && row < r.size(); ++row)
                for (std::size_t col = 0; col < 3 && col < r[row].size(); ++col)
                    R(static_cast<int>(row), static_cast<int>(col)) =
                        r[row][col].get<double>();
            const auto& tnode = node["extrinsics"]["t"];
            for (std::size_t ti = 0; ti < 3 && ti < tnode.size(); ++ti)
                t[static_cast<int>(ti)] = tnode[ti].get<double>();

            cfg.cameras.emplace_back(CameraExtrinsics{R, t}, MakeCameraModel("kb", kb));
            cfg.camera_names.push_back(name);

            const std::string full_path = image_dir + "/" + image;
            cv::Mat img = cv::imread(full_path, cv::IMREAD_COLOR);
            if (img.empty()) {
                std::cerr << "Failed to load " << full_path << "\n";
                return {};
            }
            std::cout << "  " << name << ": " << full_path << " (" << img.cols
                      << "x" << img.rows << ")\n";
            cfg.images.push_back(img);
        }

        if (!doc.contains("car") || doc["car"].empty()) {
            std::cerr << "calibration.json must contain a 'car' block with positive "
                      << "length/width — required to derive bowl params\n";
            return {};
        }
        const auto& car_node = doc["car"];
        const double car_length = car_node.value("length", 0.0);
        const double car_width = car_node.value("width", 0.0);
        if (car_length <= 0.0 || car_width <= 0.0) {
            std::cerr << "calibration.json must contain a 'car' block with positive "
                      << "length/width — required to derive bowl params\n";
            return {};
        }

        cfg.car_origin_convention = car_node.value("origin_convention", "");
        cfg.axis_x = car_node.value("axis_x", 0.0);
        cfg.bowl = BowlParams::ForCarSize(car_length, car_width, cfg.axis_x);
        return cfg;
    } catch (const nlohmann::json::exception&) {
        std::cerr << "Failed to parse JSON: " << json_path << "\n";
        return {};
    } catch (const std::exception&) {
        std::cerr << "Failed to parse JSON: " << json_path << "\n";
        return {};
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string data_dir = "../../examples/data_8";
    if (argc > 1) {
        data_dir = argv[1];
    }

    std::string json_path = data_dir + "/calibration.json";
    auto cfg = LoadViewerConfig(json_path, data_dir);
    if (cfg.cameras.empty()) {
        std::cerr << "No cameras loaded from " << json_path << "\n";
        return 1;
    }

    std::cout << "Loaded " << cfg.cameras.size() << " cameras from " << json_path << "\n";

    const cv::Vec2d car_center = cfg.bowl.CarCenter();
    std::cout << "Car origin: " << cfg.car_origin_convention << " (axis_x=" << cfg.axis_x
              << " → car_center=(" << car_center[0] << ", "
              << car_center[1] << "))\n";
    std::cout << "Derived bowl params: OuterRadius=" << cfg.bowl.OuterRadius()
              << " FlatPlatformRadius=" << cfg.bowl.FlatPlatformRadius()
              << " GridStep=" << cfg.bowl.GridStep()
              << " SlopeCoefficient=" << cfg.bowl.SlopeCoefficient()
              << " HeightExponent=" << cfg.bowl.HeightExponent()
              << " MaxHeight=" << cfg.bowl.MaxHeight() << "\n";

    BowlModel bowl(cfg.bowl, cfg.cameras);
    std::cout << "Bowl model: " << bowl.vertex_count() << " vertices, "
              << bowl.triangle_count() << " triangles\n";

    StitchScene scene(bowl, cfg.cameras[0].Width(), cfg.cameras[0].Height());
    scene.UploadCameraImages(cfg.images);

    OSGViewRenderer renderer(scene, cfg.bowl, 960, 640, RenderMode::Windowed);

    std::cout << "Controls:  LMB=rotate  RMB=zoom  MMB=pan\n";
    renderer.WaitForWindowClose();
    return 0;
}
