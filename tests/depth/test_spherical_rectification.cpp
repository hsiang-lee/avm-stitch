
#include <gtest/gtest.h>

#include "depth/spherical_rectification.hpp"

#include "test_support/depth_test_helpers.hpp"

#include <cmath>
#include <random>

using namespace avm_stitch;
using namespace avm_stitch::depth;
using avm_stitch::test::MakeStereoPair;

namespace {

double WrapPhi(double phi) {
    return phi <= -180.0 ? phi + 360.0 : phi;
}

double AngularDistance(double p, double q) {
    double d = std::fabs(WrapPhi(p) - WrapPhi(q));
    return std::min(d, 360.0 - d);
}

}
TEST(SphericalRectificationTest, DetIsRightHanded) {
            auto [a, b] = MakeStereoPair();
    auto basis = MakeRectificationBasis(a, b);
    EXPECT_NEAR(1.0, cv::determinant(basis.m1), 1e-9);
    EXPECT_NEAR(1.0, cv::determinant(basis.m2), 1e-9);
}

TEST(SphericalRectificationTest, BasisIsOrthonormal) {
    auto [a, b] = MakeStereoPair();
    auto basis = MakeRectificationBasis(a, b);
    cv::Matx33d eye = cv::Matx33d::eye();

    cv::Matx33d id1 = basis.m1 * basis.m1.t();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(eye(i, j), id1(i, j), 1e-9);

    cv::Matx33d id2 = basis.m2 * basis.m2.t();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(eye(i, j), id2(i, j), 1e-9);
}

TEST(SphericalRectificationTest, BaselinePointsFromAToB) {
            auto [a, b] = MakeStereoPair();
    auto basis = MakeRectificationBasis(a, b);
    cv::Vec3d v = a.RotateWorldToCamera(b.position() - a.position());
    v /= cv::norm(v);
    EXPECT_GT(basis.e1.dot(v), 0.999);
}

TEST(SphericalRectificationTest, RowAlignmentHoldsForRandomPoints) {
                auto [a, b] = MakeStereoPair();
    auto basis = MakeRectificationBasis(a, b);

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> xy(-0.9, 0.9);
    std::uniform_real_distribution<double> zf(0.1, 1.0);
    std::uniform_real_distribution<double> range(1.0, 30.0);

    for (int i = 0; i < 50; ++i) {
        cv::Vec3d d_a(xy(rng), xy(rng), zf(rng));
        d_a /= cv::norm(d_a);
        cv::Vec3d P = a.position() + range(rng) * (a.RotateCameraToWorld(d_a));

        cv::Vec3d d_a_w = P - a.position();
        d_a_w /= cv::norm(d_a_w);
        cv::Vec3d d_b_w = P - b.position();
        d_b_w /= cv::norm(d_b_w);

        double phi_a = PhiDeg(RectifiedDirection(basis, d_a_w, a, false));
        double phi_b = PhiDeg(RectifiedDirection(basis, d_b_w, b, true));
        EXPECT_LT(AngularDistance(phi_a, phi_b), 1e-9);
    }
}

TEST(SphericalRectificationTest, GridDirectionIsUnit) {
    SphericalGrid g;      EXPECT_NEAR(0.0, g.PsiDeg(0), 1e-12);
    EXPECT_NEAR(180.0, g.PsiDeg(g.width - 1), 1e-12);
    EXPECT_NEAR(-180.0, g.PhiDeg(0), 1e-12);
    EXPECT_NEAR(180.0, g.PhiDeg(g.height - 1), 1e-12);

    for (int r : {0, g.height / 2, g.height - 1})
        for (int c : {0, g.width / 2, g.width - 1})
            EXPECT_NEAR(1.0, cv::norm(g.DirectionC(r, c)), 1e-12);
}

TEST(SphericalRectificationTest, MirrorKeepsBasisRightHanded) {
            auto [a, b] = MakeStereoPair();
    auto basis = MakeRectificationBasis(a, b);
    EXPECT_NEAR(1.0, cv::determinant(basis.m1), 1e-9);

    SphericalGrid g;
    SphericalGrid gm = g;
    gm.mirror_rows = true;
    EXPECT_NEAR(180.0, gm.PhiDeg(0), 1e-12);
    for (int r = 0; r < g.height; ++r) {
        EXPECT_NEAR(-g.PhiDeg(r), gm.PhiDeg(r), 1e-9);
    }
        for (int r : {0, g.height / 2, g.height - 1})
        for (int c : {0, g.width / 2, g.width - 1})
            EXPECT_NEAR(1.0, cv::norm(gm.DirectionC(r, c)), 1e-12);
}

TEST(SphericalRectificationTest, SkyUpNeedsMirrorDetectsFlippedRowOrder) {
                auto [a, b] = MakeStereoPair();
    auto basis = MakeRectificationBasis(a, b);
    const int H = 720;
    const double step = 360.0 / (H - 1);

    bool need_mirror = SkyUpNeedsMirror(basis, a, H);
    cv::Vec3d sky(0.0, 0.0, 1.0);
    double phi_sky = PhiDeg(RectifiedDirection(basis, sky, a, false));
    double row_unmir = (phi_sky + 180.0) / step;

    if (need_mirror) {
        EXPECT_GE(row_unmir, H * 0.5);                         double row_mir = (-phi_sky + 180.0) / step;            EXPECT_GE(row_mir, 0.0);
        EXPECT_LT(row_mir, H * 0.5);                       } else {
        EXPECT_GE(row_unmir, 0.0);
        EXPECT_LT(row_unmir, H * 0.5);                     }
}
