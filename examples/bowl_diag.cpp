// bowl_diag.cpp — TEMPORARY diagnostic tool (read-only, do not commit).
// Root-cause check for "8-camera ship data renders as a flat plane, not a bowl".
//
// For a given calibration.json it builds BowlModel under:
//   A) the production derivation (DeriveBowlParams from the car block)
//   B) three candidate scaled parameter sets
// and prints per-model statistics: vertex/triangle counts, z distribution,
// xy extent, mesh radius vs camera xy positions, and per-camera projection
// coverage over the retained vertices.
//
// Usage: ./bowl_diag [calibration.json]
//        (default: /home/leehsiang/Documents/avm-studio/avm-stitch/examples/data_8/calibration.json)

#include "geometry/bowl_model.hpp"
#include "geometry/blend_weights.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
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

// Same parsing as avm_viewer.cpp (copy — avm_viewer.cpp itself must stay untouched).
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

struct CarBlock {
    bool present = false;
    double length = 5.6, width = 2.2, axis_x = 0.0;
};

CarBlock LoadCarBlock(const std::string& json_path) {
    CarBlock car;
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) return car;
    std::ostringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();
    if (content.empty() || content.back() != '\n') content += '\n';
    cv::FileStorage fs(content, cv::FileStorage::READ |
                                    cv::FileStorage::MEMORY |
                                    cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) return car;
    cv::FileNode car_node = fs["car"];
    if (car_node.empty()) return car;
    car.present = true;
    car.length = static_cast<double>(car_node["length"]);
    car.width = static_cast<double>(car_node["width"]);
    car.axis_x = static_cast<double>(car_node["axis_x"]);
    return car;
}

struct ModelStats {
    int vertices = 0, triangles = 0;
    double z_min = 0, z_max = 0, z_mean = 0;
    double frac_z_gt_01 = 0;         // fraction of vertices with z > 0.1
    double x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    double max_radius = 0;           // max xy-distance from car_center over retained vertices
    double frac_weighted = 0;        // fraction of vertices with any nonzero baked weight
    std::vector<double> z_band_frac; // 4 bands: [0,max_z/4),[max_z/4,max_z/2),[max_z/2,3max_z/4),[3max_z/4,max_z]
    std::vector<double> cam_dist;    // per camera |t_xy - car_center|
    std::vector<double> cam_vis;     // per camera fraction of vertices passing full visibility gate
    std::vector<int> cam_weighted;   // per camera vertices with weight > 1e-9
};

ModelStats Diagnose(const std::string& label, const BowlParams& bp,
                    const std::vector<CameraIntrinsics>& intrs,
                    const std::vector<CameraExtrinsics>& extrs) {
    const int N = static_cast<int>(intrs.size());
    const cv::Vec2d cc = bp.car_center;

    std::cout << "\n=== " << label << " ===\n";
    std::cout << "params: r_outter=" << bp.r_outter << " round_l=" << bp.round_l
              << " k_slop=" << bp.k_slop << " z_exp=" << bp.z_exp
              << " max_z=" << bp.max_z << " step=" << bp.step
              << " car=" << bp.car_length << "x" << bp.car_width
              << " car_center=(" << cc[0] << "," << cc[1] << ")\n";
    // rd* = (max_z/k_slop)^(1/z_exp): rd at which z hits max_z and the
    // `if (z > max_z) continue` clip starts dropping vertices.
    double rd_star = (bp.max_z / bp.k_slop) > 0
                         ? std::pow(bp.max_z / bp.k_slop, 1.0 / bp.z_exp)
                         : 0.0;
    std::cout << "rd* (z hits max_z) = " << rd_star
              << "   | grid half-extent = r_outter = " << bp.r_outter
              << "   | rows x cols = " << 2 * static_cast<int>(std::lround(bp.r_outter / bp.step)) << "\n";

    BowlModel bowl(bp, intrs, extrs);

    ModelStats st;
    st.vertices = bowl.vertex_count();
    st.triangles = bowl.triangle_count();

    double z_sum = 0;
    int z_gt01 = 0, weighted = 0;
    st.z_band_frac.assign(4, 0.0);
    const double band = bp.max_z / 4.0;

    st.x_min = st.y_min = 1e300;
    st.x_max = st.y_max = -1e300;

    st.cam_dist.assign(N, 0.0);
    st.cam_vis.assign(N, 0.0);
    st.cam_weighted.assign(N, 0);
    std::vector<int> cam_vis_count(N, 0);

    for (int i = 0; i < st.vertices; ++i) {
        const cv::Vec3d& P = bowl.vertex(i);
        const double z = P[2];
        z_sum += z;
        if (z > 0.1) ++z_gt01;
        st.x_min = std::min(st.x_min, P[0]);
        st.x_max = std::max(st.x_max, P[0]);
        st.y_min = std::min(st.y_min, P[1]);
        st.y_max = std::max(st.y_max, P[1]);
        st.z_min = std::min(st.z_min, z);
        st.z_max = std::max(st.z_max, z);
        st.max_radius = std::max(st.max_radius,
                                 std::hypot(P[0] - cc[0], P[1] - cc[1]));
        for (int b = 0; b < 4; ++b)
            if (z >= b * band && z < (b + 1) * band) { st.z_band_frac[b] += 1; break; }
        if (z >= bp.max_z) st.z_band_frac[3] += 1;

        double wsum = 0.0;
        const BakedVertex& bv = bowl.baked_vertex(i);
        for (int j = 0; j < N; ++j) {
            wsum += bv.weight[j];
            if (bv.weight[j] > 1e-9) st.cam_weighted[j]++;
        }
        if (wsum > 1e-9) ++weighted;

        // Independent visibility check (same criterion as BakeVertices:
        // image rect + fisheye circle + hard edge feather > 0).
        for (int j = 0; j < N; ++j) {
            double u_px, v_px;
            ProjectWorldToFishEye(P, intrs[j], extrs[j], u_px, v_px);
            bool vis = u_px >= 0.0 && u_px < intrs[j].width &&
                       v_px >= 0.0 && v_px < intrs[j].height &&
                       IsInsideFisheyeCircle(intrs[j], u_px, v_px) &&
                       FisheyeEdgeFeather(intrs[j], u_px, v_px) > 0.0;
            if (vis) cam_vis_count[j]++;
        }
    }

    if (st.vertices > 0) {
        st.z_mean = z_sum / st.vertices;
        st.frac_z_gt_01 = static_cast<double>(z_gt01) / st.vertices;
        st.frac_weighted = static_cast<double>(weighted) / st.vertices;
        for (int b = 0; b < 4; ++b)
            st.z_band_frac[b] /= st.vertices;
        for (int j = 0; j < N; ++j) {
            st.cam_vis[j] = static_cast<double>(cam_vis_count[j]) / st.vertices;
            st.cam_dist[j] = std::hypot(extrs[j].t[0] - cc[0], extrs[j].t[1] - cc[1]);
        }
    }

    std::cout << "mesh: vertices=" << st.vertices
              << " triangles=" << st.triangles << "\n";
    std::cout << "z: min=" << st.z_min << " max=" << st.z_max
              << " mean=" << st.z_mean
              << "  frac(z>0.1)=" << st.frac_z_gt_01 * 100.0 << "%"
              << "  frac(any camera weight)=" << st.frac_weighted * 100.0 << "%\n";
    std::cout << "z bands (of max_z=" << bp.max_z << "):"
              << " [0," << band << ")=" << st.z_band_frac[0] * 100.0 << "%"
              << " [" << band << "," << 2 * band << ")=" << st.z_band_frac[1] * 100.0 << "%"
              << " [" << 2 * band << "," << 3 * band << ")=" << st.z_band_frac[2] * 100.0 << "%"
              << " [" << 3 * band << "," << bp.max_z << "]=" << st.z_band_frac[3] * 100.0 << "%\n";
    std::cout << "xy bbox: x=[" << st.x_min << "," << st.x_max << "]"
              << " y=[" << st.y_min << "," << st.y_max << "]"
              << "  max_radius_from_car_center=" << st.max_radius << "\n";
    std::cout << "cameras (t vs mesh reach):\n";
    for (int j = 0; j < N; ++j) {
        std::cout << "  " << j << ": t=(" << extrs[j].t[0] << "," << extrs[j].t[1]
                  << "," << extrs[j].t[2] << ")"
                  << " |t_xy-cc|=" << st.cam_dist[j]
                  << (st.cam_dist[j] <= st.max_radius ? "  [inside mesh radius]" : "  [OUTSIDE mesh radius]")
                  << " vis_frac=" << st.cam_vis[j] * 100.0 << "%"
                  << " weighted_verts=" << st.cam_weighted[j] << "\n";
    }
    return st;
}

// ---------------------------------------------------------------------------
// Shape profile: BowlModel only stores mesh vertices, so query the analytic
// surface directly (BowlModel::ComputeRd + z = k_slop * rd^z_exp, the exact
// formula GenerateBlockBoatMesh uses) to objectively characterise the shape.
// ---------------------------------------------------------------------------

struct ProfilePoint {
    double rd = 0, z = 0;
    bool dropped = false;  // rd > r_outter: outside grid clip, no vertex
    bool clipped = false;  // z > max_z: mesh generator drops the vertex
};

ProfilePoint SampleBowl(double x, double y, const BowlParams& bp) {
    ProfilePoint p;
    p.rd = BowlModel::ComputeRd(x, y, bp);
    p.dropped = p.rd > bp.r_outter + 1e-9;
    p.z = p.rd > 0.0 ? bp.k_slop * std::pow(p.rd, bp.z_exp) : 0.0;
    p.clipped = p.z > bp.max_z;
    return p;
}

void Profile(const std::string& label, const BowlParams& bp,
             const std::vector<CameraExtrinsics>& extrs) {
    const cv::Vec2d cc = bp.car_center;
    const int N = static_cast<int>(extrs.size());

    std::cout << "\n=== SHAPE PROFILE " << label << " ===\n";
    const std::streamsize old_prec = std::cout.precision();
    std::cout << std::fixed << std::setprecision(2);

    // a) longitudinal: y = cc_y, x through bow/stern, step r_outter/20.
    std::cout << "-- a) longitudinal profile (y=cc_y; x = bow/stern axis) --\n";
    const int ns = 20;
    for (int i = 0; i <= 2 * ns; ++i) {
        double rel = -bp.r_outter + bp.r_outter * i / ns;
        auto p = SampleBowl(cc[0] + rel, cc[1], bp);
        std::cout << "   x=" << std::setw(9) << cc[0] + rel
                  << " rel_x=" << std::setw(9) << rel
                  << " rd=" << std::setw(9) << p.rd
                  << " z=" << std::setw(9) << p.z
                  << (p.clipped ? " CLIP" : "    ")
                  << (p.dropped ? " DROP" : "") << "\n";
    }

    // b) lateral: x = cc_x, y through the two ship sides.
    std::cout << "-- b) lateral profile (x=cc_x; y = side axis) --\n";
    for (int i = 0; i <= 2 * ns; ++i) {
        double rel = -bp.r_outter + bp.r_outter * i / ns;
        auto p = SampleBowl(cc[0], cc[1] + rel, bp);
        std::cout << "   y=" << std::setw(9) << cc[1] + rel
                  << " rel_y=" << std::setw(9) << rel
                  << " rd=" << std::setw(9) << p.rd
                  << " z=" << std::setw(9) << p.z
                  << (p.clipped ? " CLIP" : "    ")
                  << (p.dropped ? " DROP" : "") << "\n";
    }

    // c) radial: z at r = k/10 * r_outter for 8 angles (0deg = +x).
    std::cout << "-- c) radial profile: z at r = k/10 * r_outter ('*' = CLIP) --\n";
    std::cout << "    r\\deg";
    for (int a = 0; a < 8; ++a) std::cout << std::setw(9) << a * 45;
    std::cout << "\n";
    for (int k = 1; k <= 10; ++k) {
        double r = bp.r_outter * k / 10.0;
        std::cout << "  " << std::setw(7) << r;
        for (int a = 0; a < 8; ++a) {
            double ang = M_PI * a / 4.0;
            auto p = SampleBowl(cc[0] + r * std::cos(ang),
                                cc[1] + r * std::sin(ang), bp);
            std::cout << std::setw(8) << p.z << (p.clipped ? "*" : " ");
        }
        std::cout << "\n";
    }
    // Per-angle flat-zone reach and clip-onset radius (scan, step r_outter/2000).
    std::cout << "   per-angle: flat-until-r (rd first >0) / clip-from-r (z first >max_z):\n";
    const int scan = 2000;
    for (int a = 0; a < 8; ++a) {
        double ang = M_PI * a / 4.0;
        double onset = -1, clip_r = -1;
        for (int i = 1; i <= scan; ++i) {
            double r = bp.r_outter * i / scan;
            auto p = SampleBowl(cc[0] + r * std::cos(ang),
                                cc[1] + r * std::sin(ang), bp);
            if (onset < 0 && p.rd > 0) onset = r;
            if (p.clipped) { clip_r = r; break; }
        }
        std::cout << "     deg=" << std::setw(3) << a * 45
                  << "  flat_until_r=" << std::setw(8) << onset
                  << "  clip_from_r=" << std::setw(8) << clip_r << "\n";
    }

    // d) flat region vs grid vs camera ring scale comparison.
    const double half_len = bp.car_length / 2.0 + bp.round_l;
    const double half_wid = bp.car_width / 2.0 + bp.round_l;
    std::cout << "-- d) scale comparison --\n";
    std::cout << "   flat half-length (x): " << std::setw(8) << half_len
              << " = " << half_len / bp.r_outter * 100.0 << "% of r_outter\n";
    std::cout << "   flat half-width  (y): " << std::setw(8) << half_wid
              << " = " << half_wid / bp.r_outter * 100.0 << "% of r_outter\n";
    std::cout << "   grid half-extent    : " << std::setw(8) << bp.r_outter << " (= r_outter)\n";
    std::cout << "   max_z=" << bp.max_z << " k_slop=" << std::setprecision(6)
              << bp.k_slop << std::setprecision(2) << " z_exp=" << bp.z_exp << "\n";
    for (int j = 0; j < N; ++j) {
        double relx = extrs[j].t[0] - cc[0], rely = extrs[j].t[1] - cc[1];
        double r = std::hypot(relx, rely);
        double deg = std::atan2(rely, relx) * 180.0 / M_PI;
        auto p = SampleBowl(extrs[j].t[0], extrs[j].t[1], bp);
        std::cout << "   cam " << j << ": r_xy=" << std::setw(8) << r
                  << " (" << r / bp.r_outter * 100.0 << "% of r_outter)"
                  << " angle=" << std::setw(8) << deg
                  << " rd=" << std::setw(8) << p.rd
                  << " z_at_cam_xy=" << std::setw(8) << p.z
                  << (p.clipped ? " CLIP" : "")
                  << (p.rd > 0.0 ? "  [WALL]" : "  [FLAT]") << "\n";
    }

    // e) optical-axis orientation. T_wc convention (projection.hpp:
    // P_cam = R^T (P - t)): camera +Z (optical axis) in world = 3rd column of R.
    std::cout << "-- e) camera optical axis (R col2) vs direction-to-car-center (ground z=0) --\n";
    for (int j = 0; j < N; ++j) {
        cv::Vec3d fwd(extrs[j].R(0, 2), extrs[j].R(1, 2), extrs[j].R(2, 2));
        double flen = std::sqrt(fwd.dot(fwd));
        if (flen > 1e-9) fwd /= flen;
        cv::Vec3d to_c(cc[0] - extrs[j].t[0], cc[1] - extrs[j].t[1], 0.0 - extrs[j].t[2]);
        double tlen = std::sqrt(to_c.dot(to_c));
        if (tlen > 1e-9) to_c /= tlen;
        double dot = fwd.dot(to_c);
        cv::Vec3d fh(fwd[0], fwd[1], 0.0);
        cv::Vec3d th(to_c[0], to_c[1], 0.0);
        double dot_h = std::sqrt(fh.dot(fh)) > 1e-9 && std::sqrt(th.dot(th)) > 1e-9
                           ? fh.dot(th) / (std::sqrt(fh.dot(fh)) * std::sqrt(th.dot(th)))
                           : 0.0;
        std::cout << "   cam " << j << ": fwd=(" << std::setw(7) << fwd[0] << ","
                  << std::setw(7) << fwd[1] << "," << std::setw(7) << fwd[2] << ")"
                  << " dot=" << std::setw(7) << dot
                  << " dot_xy=" << std::setw(7) << dot_h
                  << (dot > 0.0 ? "  [INWARD]" : "  [OUTWARD]")
                  << (dot_h > 0.0 ? "[xy-INWARD]" : "[xy-OUTWARD]") << "\n";
    }

    std::cout.unsetf(std::ios::fixed);
    std::cout.precision(old_prec);
}

} // namespace

int main(int argc, char** argv) {
    std::string json_path =
        "/home/leehsiang/Documents/avm-studio/avm-stitch/examples/data_8/calibration.json";
    if (argc > 1) json_path = argv[1];

    auto cams = LoadCameraConfig(json_path);
    if (cams.empty()) {
        std::cerr << "No cameras loaded from " << json_path << "\n";
        return 1;
    }
    CarBlock car = LoadCarBlock(json_path);

    std::cout << "data: " << json_path << "\n";
    std::cout << "cameras: " << cams.size()
              << "  car block: " << (car.present ? "present" : "ABSENT")
              << (car.present ? (" (length=" + std::to_string(car.length) +
                                 " width=" + std::to_string(car.width) +
                                 " axis_x=" + std::to_string(car.axis_x) + ")")
                              : "")
              << "\n";

    std::vector<CameraIntrinsics> intrs;
    std::vector<CameraExtrinsics> extrs;
    for (auto& cam : cams) {
        CameraIntrinsics intr;
        intr.width = cam.width;  intr.height = cam.height;
        intr.fx = cam.fx;        intr.fy = cam.fy;
        intr.cx = cam.cx;        intr.cy = cam.cy;
        intr.k1 = cam.k1;        intr.k2 = cam.k2;
        intr.k3 = cam.k3;        intr.k4 = cam.k4;
        intrs.push_back(intr);
        extrs.push_back({cam.R, cam.t});   // T_wc (camera->world), same as avm_viewer
    }

    // A) The production derivation: all parameters from the car block dims.
    {
        BowlParams bp = DeriveBowlParams(car.length, car.width, car.axis_x);
        if (!car.present) {
            std::cerr << "calibration has no 'car' block — cannot derive bowl params\n";
            return 1;
        }
        Diagnose("A: DeriveBowlParams(car_length, car_width, axis_x)", bp, intrs, extrs);
        Profile("A: DeriveBowlParams(car_length, car_width, axis_x)", bp, extrs);
    }

    // B) Candidates: step scaled with r_outter to keep rows*cols bounded.
    struct Cand { const char* name; double r_outter, round_l, k_slop, z_exp, max_z, step; };
    const Cand candidates[] = {
        {"B1: r_outter=120 round_l=20 k=0.0009 exp=2.2 max_z=60 step=1.0",
         120, 20, 0.0009, 2.2, 60, 1.0},
        {"B2: r_outter=110 round_l=15 k=0.001 exp=2.0 max_z=40 step=1.0",
         110, 15, 0.001, 2.0, 40, 1.0},
        {"B3: r_outter=100 round_l=10 k=0.002 exp=2.2 max_z=50 step=0.5",
         100, 10, 0.002, 2.2, 50, 0.5},
    };
    for (const auto& c : candidates) {
        // Explicit candidate params over the derived base (bowl_diag is a
        // diagnostic: every field of the tested parameter set is explicit).
        BowlParams bp = DeriveBowlParams(car.length, car.width, car.axis_x);
        bp.r_outter = c.r_outter;
        bp.round_l = c.round_l;
        bp.k_slop = c.k_slop;
        bp.z_exp = c.z_exp;
        bp.max_z = c.max_z;
        bp.step = c.step;
        Diagnose(c.name, bp, intrs, extrs);
    }

    std::cout << "\n=== done ===\n";
    return 0;
}
