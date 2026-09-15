// avm_viewer — Textured bowl-model viewer.
// Loads real fish-eye camera data from calibration.json and renders a textured bowl.
// Defaults to ../../examples/data_8 (8 cameras); pass another data dir to override,
// e.g. ./avm_viewer ../../examples/data_4 (4 cameras).

#include "geometry/bowl_model.hpp"
#include "render/osg_view_renderer.hpp"
#include "render/stitch_scene.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace avm_stitch;

namespace {

struct CameraData {
    std::string name;
    std::string image_path;
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double k1 = 0, k2 = 0, k3 = 0, k4 = 0;
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t = cv::Vec3d::zeros();
};

std::vector<CameraData> LoadCameraConfig(const std::string& json_path) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open " << json_path << "\n";
        return {};
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();
    if (content.empty() || content.back() != '\n') content += '\n';

    cv::FileStorage fs(content, cv::FileStorage::READ |
                                cv::FileStorage::MEMORY |
                                cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {
        std::cerr << "Failed to parse JSON: " << json_path << "\n";
        return {};
    }

    std::vector<CameraData> cameras;
    cv::FileNode cams = fs["cameras"];
    for (auto it = cams.begin(); it != cams.end(); ++it) {
        const cv::FileNode& node = *it;
        CameraData cam;
        cam.name = static_cast<std::string>(node["name"]);
        cam.image_path = static_cast<std::string>(node["image"]);

        const cv::FileNode& intr = node["intrinsics"];
        cam.width = static_cast<int>(node["width"]);
        cam.height = static_cast<int>(node["height"]);
        cam.fx = static_cast<double>(intr["fx"]);
        cam.fy = static_cast<double>(intr["fy"]);
        cam.cx = static_cast<double>(intr["cx"]);
        cam.cy = static_cast<double>(intr["cy"]);
        cam.k1 = static_cast<double>(intr["k1"]);
        cam.k2 = static_cast<double>(intr["k2"]);
        cam.k3 = static_cast<double>(intr["k3"]);
        cam.k4 = static_cast<double>(intr["k4"]);

        // cv::read(FileNode, Mat) only understands OpenCV's named
        // {dt,rows,cols,data} matrix layout, not plain nested JSON arrays,
        // so walk the sequences directly (still fully scoped parsing).
        int row = 0;
        cv::FileNode rnode = node["extrinsics"]["r"];
        for (auto rit = rnode.begin(); rit != rnode.end() && row < 3; ++rit) {
            int col = 0;
            const cv::FileNode& rownode = *rit;
            for (auto cit = rownode.begin(); cit != rownode.end() && col < 3; ++cit)
                cam.R(row, col++) = static_cast<double>(*cit);
            ++row;
        }
        int ti = 0;
        cv::FileNode tnode = node["extrinsics"]["t"];
        for (auto tit = tnode.begin(); tit != tnode.end() && ti < 3; ++tit)
            cam.t[ti++] = static_cast<double>(*tit);

        cameras.push_back(cam);
    }
    return cameras;
}

} // namespace

int main(int argc, char** argv) {
    std::string data_dir = "../../examples/data_8";
    if (argc > 1) data_dir = argv[1];

    std::string json_path = data_dir + "/calibration.json";
    auto cameras = LoadCameraConfig(json_path);
    if (cameras.empty()) {
        std::cerr << "No cameras loaded from " << json_path << "\n";
        return 1;
    }

    int N = static_cast<int>(cameras.size());
    std::cout << "Loaded " << N << " cameras from " << json_path << "\n";

    std::vector<CameraIntrinsics> intrs;
    std::vector<CameraExtrinsics> extrs;
    std::vector<cv::Mat> images;

    for (auto& cam : cameras) {
        CameraIntrinsics intr;
        intr.width = cam.width;
        intr.height = cam.height;
        intr.fx = cam.fx;
        intr.fy = cam.fy;
        intr.cx = cam.cx;
        intr.cy = cam.cy;
        intr.k1 = cam.k1;
        intr.k2 = cam.k2;
        intr.k3 = cam.k3;
        intr.k4 = cam.k4;
        intrs.push_back(intr);

        // calibration.json already stores T_wc (camera→world) — same format as
        // avm-worker's OSG backend. Pass directly.
        extrs.push_back({cam.R, cam.t});

        std::string img_path = data_dir + "/" + cam.image_path;
        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "Failed to load " << img_path << "\n";
            return 1;
        }
        std::cout << "  " << cam.name << ": " << img_path << " (" << img.cols
                  << "x" << img.rows << ")\n";
        images.push_back(img);
    }

    // Read the mandatory "car" block from calibration.json. Fields: length (m),
    // width (m), axis_x (m, rear-axle distance from origin), origin_convention
    // (descriptive string). Bowl parameters are derived from these — there is
    // no default vehicle, so a missing car block is a hard error.
    cv::Vec2d car_center(0.0, 0.0);
    double car_length = 0.0;
    double car_width = 0.0;
    bool car_ok = false;
    // Re-parse the JSON content to read the "car" block (FileStorage is
    // single-pass; we need a second parse from the original content).
    std::ifstream ifs2(json_path);
    if (ifs2.is_open()) {
        std::ostringstream ss2;
        ss2 << ifs2.rdbuf();
        std::string content2 = ss2.str();
        if (content2.empty() || content2.back() != '\n') content2 += '\n';

        cv::FileStorage fs2(content2, cv::FileStorage::READ |
                                      cv::FileStorage::MEMORY |
                                      cv::FileStorage::FORMAT_JSON);
        if (fs2.isOpened()) {
            cv::FileNode car_node = fs2["car"];
            if (!car_node.empty()) {
                car_length = static_cast<double>(car_node["length"]);
                car_width = static_cast<double>(car_node["width"]);
                double axis_x = static_cast<double>(car_node["axis_x"]);
                car_center = cv::Vec2d(axis_x / 2.0, 0.0);
                std::string convention = static_cast<std::string>(car_node["origin_convention"]);
                std::cout << "Car origin: " << convention << " (axis_x=" << axis_x
                          << " → car_center=(" << car_center[0] << ", "
                          << car_center[1] << "))\n";
                car_ok = car_length > 0.0 && car_width > 0.0;
            }
        }
    }
    if (!car_ok) {
        std::cerr << "calibration.json must contain a 'car' block with positive "
                  << "length/width — required to derive bowl params\n";
        return 1;
    }

    BowlParams bp = DeriveBowlParams(car_length, car_width, car_center[0] * 2.0);
    std::cout << "Derived bowl params: r_outter=" << bp.r_outter
              << " round_l=" << bp.round_l << " step=" << bp.step
              << " k_slop=" << bp.k_slop << " z_exp=" << bp.z_exp
              << " max_z=" << bp.max_z << "\n";
    BowlModel bowl(bp, intrs, extrs);
    std::cout << "Bowl model: " << bowl.vertex_count() << " vertices, "
              << bowl.triangle_count() << " triangles\n";

    StitchScene scene(bowl, intrs[0].width, intrs[0].height);
    scene.UploadCameraImages(images);

    OSGViewRenderer renderer(scene, bp.r_outter,
                             DefaultViewPoses(bp.car_center, bp.r_outter / kCalibratedAnchorRoutter),
                             960, 640, RenderMode::Windowed, bp.car_center);

    std::cout << "Controls:  LMB=rotate  RMB=zoom  MMB=pan\n";
    renderer.WaitForWindowClose();
    return 0;
}
