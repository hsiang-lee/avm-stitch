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

struct Config {
    std::vector<Camera> cameras;
    std::vector<cv::Mat> images;
    BowlParams bowl;
};

bool LoadConfig(const std::string& json_path, const std::string& image_dir, Config& cfg) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) { std::cerr << "cannot open " << json_path << "\n"; return false; }
    std::ostringstream ss; ss << ifs.rdbuf();
    std::string content = ss.str();
    if (content.empty() || content.back() != '\n') content += '\n';

    nlohmann::json doc;
    try { doc = nlohmann::json::parse(content); }
    catch (const std::exception&) { std::cerr << "bad json\n"; return false; }

    for (const auto& node : doc["cameras"]) {
        const auto& intr = node["intrinsics"];
        KbParams kb{node.value("width", 0), node.value("height", 0),
                    intr.value("fx", 0.0), intr.value("fy", 0.0),
                    intr.value("cx", 0.0), intr.value("cy", 0.0),
                    intr.value("k1", 0.0), intr.value("k2", 0.0),
                    intr.value("k3", 0.0), intr.value("k4", 0.0)};
        cv::Matx33d R = cv::Matx33d::eye();
        cv::Vec3d t = cv::Vec3d::zeros();
        const auto& r = node["extrinsics"]["r"];
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) R(row, col) = r[row][col].get<double>();
        const auto& tn = node["extrinsics"]["t"];
        for (int i = 0; i < 3; ++i) t[i] = tn[i].get<double>();
        cfg.cameras.emplace_back(CameraExtrinsics{R, t}, MakeCameraModel("kb", kb));

        const std::string path = image_dir + "/" + node.value("image", "");
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty()) { std::cerr << "cannot read " << path << "\n"; return false; }
        cfg.images.push_back(img);
    }

    const auto& car = doc["car"];
    cfg.bowl = BowlParams::ForCarSize(car.value("length", 0.0), car.value("width", 0.0),
                                      car.value("axis_x", 0.0));
    return !cfg.cameras.empty();
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "usage: refrender <data_dir> <out_dir>\n"; return 2; }
    const std::string data_dir = argv[1];
    const std::string out_dir = argv[2];

    Config cfg;
    if (!LoadConfig(data_dir + "/calibration.json", data_dir, cfg)) return 1;

    std::cout << "cameras=" << cfg.cameras.size()
              << " image=" << cfg.images[0].cols << "x" << cfg.images[0].rows
              << " bowl_outer=" << cfg.bowl.OuterRadius() << "\n";

    BowlModel bowl(cfg.bowl, cfg.cameras);
    std::cout << "vertices=" << bowl.vertex_count()
              << " triangles=" << bowl.triangle_count() << "\n";

    StitchScene scene(bowl, cfg.cameras[0].Width(), cfg.cameras[0].Height());
    scene.UploadCameraImages(cfg.images);

    OSGViewRenderer renderer(scene, cfg.bowl, 960, 640, RenderMode::Offscreen);
    auto views = renderer.RenderAllViews();
    for (const auto& entry : views) {
        const std::string path = out_dir + "/" + entry.first + ".png";
        if (entry.second.empty()) { std::cerr << entry.first << " empty\n"; return 1; }
        if (!cv::imwrite(path, entry.second)) { std::cerr << "write failed " << path << "\n"; return 1; }
        std::cout << "wrote " << path << " " << entry.second.cols << "x" << entry.second.rows << "\n";
    }
    return 0;
}
