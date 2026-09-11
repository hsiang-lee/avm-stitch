// End-to-end acceptance test for black-seam (zero-weight vertex) count.
//
// Reads the real cameras.json (with the D5 car block), constructs a BowlModel
// with proper car_center / car_length / car_width, and verifies that the
// number of vertices with zero weight across all cameras is below the
// acceptance threshold (< 50 / total).
//
// This test validates the combined effect of all five design decisions
// (D1–D5): the bowl mesh geometry, sector model, fallback normalization,
// hand-written projection, and data-driven car parameters must work together
// to eliminate black wedges except for the centre car-floor physical blind
// zone.

#include <gtest/gtest.h>

#include "geometry/blend_weights.hpp"
#include "geometry/bowl_model.hpp"
#include "geometry/projection.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace avm_stitch;

namespace {

// Parse a 3x3 rotation matrix from a cv::FileNode that holds a nested
// JSON array ([[...], [...], [...]]).
cv::Matx33d ParseRotationMatrix(const cv::FileNode& rnode) {
    cv::Matx33d R;
    int row = 0;
    for (auto rit = rnode.begin(); rit != rnode.end() && row < 3; ++rit) {
        int col = 0;
        const cv::FileNode& rownode = *rit;
        for (auto cit = rownode.begin(); cit != rownode.end() && col < 3; ++cit) {
            R(row, col++) = static_cast<double>(*cit);
        }
        ++row;
    }
    return R;
}

// Parse a 3-element translation vector from a cv::FileNode (JSON array).
cv::Vec3d ParseTranslationVector(const cv::FileNode& tnode) {
    cv::Vec3d t(0.0, 0.0, 0.0);
    int i = 0;
    for (auto it = tnode.begin(); it != tnode.end() && i < 3; ++it) {
        t[i++] = static_cast<double>(*it);
    }
    return t;
}

struct E2ECamera {
    CameraIntrinsics intr;
    CameraExtrinsics extr;
};

struct E2EBowlConfig {
    double car_length = 5.6;
    double car_width = 2.2;
    cv::Vec2d car_center{0.0, 0.0};
    std::vector<E2ECamera> cameras;
};

E2EBowlConfig LoadConfigFromJson(const std::string& json_path) {
    E2EBowlConfig config;

    std::ifstream ifs(json_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open " << json_path << "\n";
        return config;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();
    if (content.empty() || content.back() != '\n') content += '\n';

    cv::FileStorage fs(content, cv::FileStorage::READ |
                                cv::FileStorage::MEMORY |
                                cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {
        std::cerr << "Failed to parse JSON\n";
        return config;
    }

    // Read car block (D5)
    cv::FileNode car_node = fs["car"];
    if (!car_node.empty()) {
        config.car_length = static_cast<double>(car_node["length"]);
        config.car_width  = static_cast<double>(car_node["width"]);
        double axis_x = static_cast<double>(car_node["axis_x"]);
        config.car_center = cv::Vec2d(axis_x / 2.0, 0.0);
        std::cout << "Car: length=" << config.car_length
                  << " width=" << config.car_width
                  << " axis_x=" << axis_x
                  << " → car_center=(" << config.car_center[0]
                  << ", " << config.car_center[1] << ")\n";
    } else {
        std::cout << "No 'car' field in cameras.json, using defaults\n";
    }

    // Read cameras
    cv::FileNode cams = fs["cameras"];
    for (auto it = cams.begin(); it != cams.end(); ++it) {
        const cv::FileNode& node = *it;
        E2ECamera cam;

        cam.intr.width  = static_cast<int>(node["width"]);
        cam.intr.height = static_cast<int>(node["height"]);

        const cv::FileNode& intr = node["intrinsics"];
        cam.intr.fx = static_cast<double>(intr["fx"]);
        cam.intr.fy = static_cast<double>(intr["fy"]);
        cam.intr.cx = static_cast<double>(intr["cx"]);
        cam.intr.cy = static_cast<double>(intr["cy"]);
        cam.intr.k1 = static_cast<double>(intr["k1"]);
        cam.intr.k2 = static_cast<double>(intr["k2"]);
        cam.intr.k3 = static_cast<double>(intr["k3"]);
        cam.intr.k4 = static_cast<double>(intr["k4"]);

        cam.extr.R = ParseRotationMatrix(node["extrinsics"]["r"]);
        cam.extr.t = ParseTranslationVector(node["extrinsics"]["t"]);

        config.cameras.push_back(cam);
    }

    std::cout << "Loaded " << config.cameras.size() << " cameras\n";
    return config;
}

} // namespace

// Acceptance test: with D1–D5 fully applied, the number of zero-weight
// vertices must be < 50 out of the total bowl vertices. Residual zeros
// are limited to the centre car-floor physical blind zone.
TEST(BlackSeamE2ETest, ZeroWeightVerticesBelowThreshold) {
    // Path to the real cameras.json — computed relative to the source tree.
    // The test binary runs from the build directory; the source-relative
    // path is passed via a compile definition.
    const std::string json_path = TEST_DATA_DIR "/cameras.json";

    E2EBowlConfig config = LoadConfigFromJson(json_path);
    ASSERT_GE(config.cameras.size(), 4u)
        << "Need at least 4 cameras for a meaningful E2E test";

    // Build intrinsics and extrinsics vectors
    std::vector<CameraIntrinsics> intrs;
    std::vector<CameraExtrinsics> extrs;
    for (const auto& cam : config.cameras) {
        intrs.push_back(cam.intr);
        extrs.push_back(cam.extr);
    }

    // Configure BowlParams with D5 car values
    BowlParams bp = BowlParams::Default();
    bp.car_length  = config.car_length;
    bp.car_width   = config.car_width;
    bp.car_center  = config.car_center;

    std::cout << "BowlParams: car_length=" << bp.car_length
              << " car_width=" << bp.car_width
              << " car_center=(" << bp.car_center[0] << ", " << bp.car_center[1] << ")\n";

    // Build the bowl model (mesh + bake)
    const BowlModel model(bp, intrs, extrs);

    const int total_vertices = model.vertex_count();
    std::cout << "Total bowl vertices: " << total_vertices << "\n";

    // Count vertices with zero weight across all camera slots
    int zero_weight_count = 0;
    int zero_visible_count = 0;   // zero-weight but some camera is geometrically visible
    int zero_invisible_count = 0; // zero-weight and no camera sees it
    for (int i = 0; i < total_vertices; ++i) {
        const cv::Vec3d& P = model.vertex(i);
        const BakedVertex& bv = model.baked_vertex(i);
        double weight_sum = 0.0;
        const int N = model.baked_num_cameras();
        for (int j = 0; j < N; ++j) {
            weight_sum += bv.weight[j];
        }
        if (weight_sum < 1e-9) {
            ++zero_weight_count;
            // Check how many cameras are geometrically visible for this vertex
            int visible_count = 0;
            for (int j = 0; j < N; ++j) {
                double u, v;
                ProjectWorldToFishEye(P, intrs[j], extrs[j], u, v);
                if (u >= 0.0 && u < intrs[j].width &&
                    v >= 0.0 && v < intrs[j].height &&
                    IsInsideFisheyeCircle(intrs[j], u, v)) {
                    ++visible_count;
                }
            }
            if (visible_count > 0) {
                ++zero_visible_count;
            } else {
                ++zero_invisible_count;
            }
        }
    }

    std::cout << "Zero-weight vertices: " << zero_weight_count << " / " << total_vertices << "\n";
    std::cout << "  - No visible cameras (physical blind zone): " << zero_invisible_count << "\n";
    std::cout << "  - Visible cameras exist but weight=0: " << zero_visible_count << "\n";

    // Debug: print first 10 zero-weight vertices with visible cameras
    // Analyze zero-weight vertices: distribution by position
    double car_l_half = bp.car_length / 2.0;
    double car_w_half = bp.car_width / 2.0;
    int inside_car = 0;    // vertex inside car rectangle
    int edge_car = 0;      // vertex on car edge but outside
    int z_nonzero = 0;     // vertex with z > 0 (on bowl rim, not flat center)
    int outside_car = 0;   // vertex well outside car rectangle
    for (int i = 0; i < total_vertices; ++i) {
        const cv::Vec3d& P = model.vertex(i);
        const BakedVertex& bv = model.baked_vertex(i);
        double ws = 0.0;
        for (int j = 0; j < 4; ++j) ws += bv.weight[j];
        if (ws >= 1e-9) continue;

        double rel_x = P[0] - bp.car_center[0];
        double rel_y = P[1] - bp.car_center[1];
        bool inside_car_rect = std::abs(rel_x) <= car_l_half && std::abs(rel_y) <= car_w_half;
        if (inside_car_rect) ++inside_car;
        else if (P[2] > 1e-9) ++z_nonzero;
        else if (std::abs(rel_x) <= car_l_half + 1.0 && std::abs(rel_y) <= car_w_half + 1.0) ++edge_car;
        else ++outside_car;
    }
    std::cout << "  - Inside car rectangle: " << inside_car << "\n";
    std::cout << "  - Near car edge (z=0): " << edge_car << "\n";
    std::cout << "  - On bowl rim (z>0): " << z_nonzero << "\n";
    std::cout << "  - Outside car region: " << outside_car << "\n";
    std::cout << "  Total blind: " << (inside_car+edge_car+z_nonzero+outside_car) << "\n";

    // Acceptance criterion: residual black wedges < 50 vertices
    // (the centre car-floor physical blind zone accounts for the rest).
    EXPECT_LT(zero_weight_count, 50);
}