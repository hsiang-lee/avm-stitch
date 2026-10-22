#include "core/stitch_engine.hpp"
#include "bowl_model/bowl_model.hpp"
#include "camera_model/kb_camera_model.hpp"
#include "camera_model/camera_extrinsics.hpp"
#include "render/stitch_scene.hpp"
#include "render/osg_view_renderer.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace avm_stitch;

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "usage: render_dump <calibration.json> <imgdir> [outdir]\n"; return 2; }
    std::string cal_path = argv[1], img_dir = argv[2], out_dir = argc > 3 ? argv[3] : img_dir;

    std::ifstream ifs(cal_path); std::ostringstream ss; ss << ifs.rdbuf();
    cv::FileStorage fs(ss.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) { std::cerr << "bad json\n"; return 1; }

    std::vector<KbParams> intrs; std::vector<CameraExtrinsics> extrs;
    std::vector<cv::Mat> imgs; std::vector<std::string> names;
    cv::FileNode cams = fs["cameras"];
    for (auto it = cams.begin(); it != cams.end(); ++it) {
        const cv::FileNode& node = *it;
        CameraExtrinsics extr;
        cv::Matx33d R;
        cv::Vec3d t;
        const cv::FileNode& rn=node["rotation"]; int k=0;
        for (auto rit=rn.begin(); rit!=rn.end() && k<9; ++rit) R(k/3,k%3)=(double)*rit, ++k;
        const cv::FileNode& tn=node["translation"]; int ki=0;
        for (auto tit=tn.begin(); tit!=tn.end() && ki<3; ++tit) t[ki++]=(double)*tit;
        extr = CameraExtrinsics{R, t}; extrs.push_back(extr);
                        intrs.push_back(KbParams{
            (int)node["width"], (int)node["height"],
            (double)node["fx"], (double)node["fy"],
            (double)node["cx"], (double)node["cy"],
            (double)node["k1"], (double)node["k2"],
            (double)node["k3"], (double)node["k4"]});
        names.push_back((std::string)node["label"]);
        cv::Mat img=cv::imread(img_dir+"/"+(std::string)node["image"], cv::IMREAD_COLOR);
        if (img.empty()) { std::cerr << "cannot load " << (std::string)node["image"] << "\n"; return 1; }
        imgs.push_back(img);
        std::cout << "cam " << names.back() << " (" << img.cols << "x" << img.rows << ")\n";
    }
    cv::FileNode car = fs["car"];
    double len=(double)car["length"], wid=(double)car["width"], ax=(double)car["axis_x"];
    BowlParams bp = BowlParams::ForCarSize(len, wid, ax);
    std::cout << "CarCenter=(" << bp.CarCenter()[0] << "," << bp.CarCenter()[1] << ") OuterRadius=" << bp.OuterRadius() << "\n";

        StitchEngine engine;
    std::vector<Camera> cameras;
    cameras.reserve(intrs.size());
    for (size_t i = 0; i < intrs.size(); ++i) {
        cameras.emplace_back(extrs[i], MakeCameraModel("kb", intrs[i]));
    }
    engine.Configure(std::move(cameras), bp, 960, 640);
    auto views = engine.RenderFrame(imgs);
    for (auto& kv : views) {
        cv::imwrite(out_dir+"/view_"+kv.first+".png", kv.second);
        std::cout << "render " << kv.first << "\n";
    }

        {
                std::vector<Camera> cams;
        for (size_t i = 0; i < intrs.size(); ++i) {
            cams.emplace_back(extrs[i], std::make_unique<KbCameraModel>(intrs[i]));
        }
        BowlModel bowl(bp, cams);
        StitchScene scene(bowl, intrs[0].width, intrs[0].height);
        scene.UploadCameraImages(imgs);
        OSGViewRenderer renderer(scene, bp, 1024, 1024, RenderMode::Offscreen);
        auto m = renderer.RenderAllViews();
        for (auto& kv : m) cv::imwrite(out_dir+"/view_"+kv.first+".png", kv.second);
        std::cout << "render 4 default views\n";
    }
    return 0;
}
