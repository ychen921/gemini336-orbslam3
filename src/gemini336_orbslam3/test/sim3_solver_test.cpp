#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapPoint.h>
#include <Pinhole.h>
#include <Sim3Solver.h>
#include <spdlog/sinks/null_sink.h>

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

// Exercise the backend implementation itself. The public getters describe the
// best RANSAC model, so expose the current outputs for direct ComputeSim3 calls.
class TestSim3Solver : public ORB_SLAM3::Sim3Solver
{
public:
    using Sim3Solver::Sim3Solver;
    using Sim3Solver::ComputeSim3;
    using Sim3Solver::mR12i;
    using Sim3Solver::ms12i;
    using Sim3Solver::mt12i;
    using Sim3Solver::mT12i;
    using Sim3Solver::mT21i;
    using Sim3Solver::mnIterations;
    using Sim3Solver::mnBestInliers;
    using Sim3Solver::mRansacMaxIts;
    using Sim3Solver::mvAllIndices;
};

constexpr std::size_t fixture_points = 6;
using FixturePoints = Eigen::Matrix<float, 3, fixture_points>;

FixturePoints make_fixture_points()
{
    FixturePoints points;
    points << -1.0f, 1.0f, 0.0f, -0.5f, 0.5f, 0.0f,
              -1.0f, -1.0f, 1.0f, 0.5f, 0.5f, 0.0f,
              4.0f, 4.0f, 4.0f, 5.0f, 5.0f, 6.0f;
    return points;
}

void prepare_frame(ORB_SLAM3::Frame& frame, ORB_SLAM3::Pinhole& camera,
                   const FixturePoints& points, unsigned long id)
{
    // KeyFrame copies these fields even though this test never extracts features
    // or runs tracking. Prepare them before invoking its real Frame constructor.
    frame.mnId = id;
    frame.mTimeStamp = static_cast<double>(id);
    frame.mnDataset = 0;
    frame.N = static_cast<int>(fixture_points);
    frame.Nleft = -1;
    frame.Nright = -1;
    frame.mpCamera = &camera;
    frame.mpCamera2 = nullptr;
    frame.mpORBvocabulary = nullptr;
    frame.mK = camera.toK();
    frame.mK_ = camera.toK_();
    frame.mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    frame.mbf = 0.0f;
    frame.mb = 0.0f;
    frame.mThDepth = 40.0f;
    frame.mnScaleLevels = 1;
    frame.mfScaleFactor = 1.2f;
    frame.mfLogScaleFactor = std::log(frame.mfScaleFactor);
    frame.mvScaleFactors = {1.0f};
    frame.mvInvScaleFactors = {1.0f};
    frame.mvLevelSigma2 = {1.0f};
    frame.mvInvLevelSigma2 = {1.0f};
    frame.mvuRight.assign(fixture_points, -1.0f);
    frame.mvDepth.resize(fixture_points);
    frame.mvpMapPoints.assign(fixture_points, nullptr);
    frame.mvbOutlier.assign(fixture_points, false);
    frame.mDescriptors = cv::Mat::zeros(frame.N, 32, CV_8U);
    frame.mImuCalib.Set(Sophus::SE3f(), 1.0f, 1.0f, 1.0f, 1.0f);
    frame.SetPose(Sophus::SE3f());

    for (std::size_t i = 0; i < fixture_points; ++i)
    {
        const Eigen::Vector3f point = points.col(i);
        const Eigen::Vector2f pixel = camera.project(point);
        frame.mvKeysUn.emplace_back(pixel.x(), pixel.y(), 1.0f);
        frame.mvDepth[i] = point.z();
    }
    frame.mvKeys = frame.mvKeysUn;
}

struct Fixture
{
    std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::logger>(
        "sim3_solver_test", std::make_shared<spdlog::sinks::null_sink_mt>());
    ORB_SLAM3::Pinhole camera{std::vector<float>{400.0f, 400.0f, 320.0f, 240.0f}};
    ORB_SLAM3::Map map{0, logger};
    ORB_SLAM3::Frame frame1;
    ORB_SLAM3::Frame frame2;
    std::unique_ptr<ORB_SLAM3::KeyFrame> keyframe1;
    std::unique_ptr<ORB_SLAM3::KeyFrame> keyframe2;
    std::array<std::unique_ptr<ORB_SLAM3::MapPoint>, fixture_points> points1;
    std::array<std::unique_ptr<ORB_SLAM3::MapPoint>, fixture_points> points2;
    std::vector<ORB_SLAM3::MapPoint*> matches;

    Fixture(const FixturePoints& positions1 = make_fixture_points(),
            const FixturePoints& positions2 = make_fixture_points())
    {
        // Frame keeps calibration/grid constants statically. Each CTest case
        // runs in a separate process, so this setup cannot affect other cases.
        ORB_SLAM3::Frame::fx = 400.0f;
        ORB_SLAM3::Frame::fy = 400.0f;
        ORB_SLAM3::Frame::cx = 320.0f;
        ORB_SLAM3::Frame::cy = 240.0f;
        ORB_SLAM3::Frame::invfx = 1.0f / 400.0f;
        ORB_SLAM3::Frame::invfy = 1.0f / 400.0f;
        ORB_SLAM3::Frame::mnMinX = 0.0f;
        ORB_SLAM3::Frame::mnMinY = 0.0f;
        ORB_SLAM3::Frame::mnMaxX = 640.0f;
        ORB_SLAM3::Frame::mnMaxY = 480.0f;
        ORB_SLAM3::Frame::mfGridElementWidthInv = FRAME_GRID_COLS / 640.0f;
        ORB_SLAM3::Frame::mfGridElementHeightInv = FRAME_GRID_ROWS / 480.0f;

        prepare_frame(frame1, camera, positions1, 10);
        prepare_frame(frame2, camera, positions2, 20);
        keyframe1 = std::make_unique<ORB_SLAM3::KeyFrame>(frame1, &map, nullptr);
        keyframe2 = std::make_unique<ORB_SLAM3::KeyFrame>(frame2, &map, nullptr);

        // Six valid observations keep construction independent of the numeric
        // sample under test and avoid the constructor's empty-RANSAC path.
        for (std::size_t i = 0; i < fixture_points; ++i)
        {
            const Eigen::Vector3f position1 = positions1.col(i);
            const Eigen::Vector3f position2 = positions2.col(i);
            points1[i] = std::make_unique<ORB_SLAM3::MapPoint>(position1, keyframe1.get(), &map);
            points2[i] = std::make_unique<ORB_SLAM3::MapPoint>(position2, keyframe2.get(), &map);
            points1[i]->AddObservation(keyframe1.get(), static_cast<int>(i));
            points2[i]->AddObservation(keyframe2.get(), static_cast<int>(i));
            keyframe1->AddMapPoint(points1[i].get(), i);
            keyframe2->AddMapPoint(points2[i].get(), i);
            matches.push_back(points2[i].get());
        }
        // Map does not own the fixtures: all objects remain under RAII ownership.
    }
};

struct Sample
{
    Eigen::Matrix3f points1;
    Eigen::Matrix3f points2;
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    float scale = 1.0f;
    float geometry_scale = 1.0f;
    bool fixed_scale = true;
    bool expect_rejection = false;
};

Sample make_sample(const std::string& name)
{
    Sample sample;
    // A non-collinear triangle with exact centroid (0, 0, 4) makes identity
    // rotation deterministic without reproducing Horn's calculation in the test.
    sample.points2 << 1.0f, -1.0f, 0.0f,
                      1.0f, 1.0f, -2.0f,
                      4.0f, 4.0f, 4.0f;

    if (name == "identity")
    {
        sample.points1 = sample.points2;
        return sample;
    }
    if (name == "translation_only")
    {
        sample.translation = Eigen::Vector3f(0.25f, -0.5f, 0.75f);
    }
    else if (name == "small_angle")
    {
        sample.rotation = Eigen::AngleAxisf(
            1.0e-4f, Eigen::Vector3f(1.0f, 2.0f, 3.0f).normalized()).toRotationMatrix();
        sample.translation = Eigen::Vector3f(0.25f, -0.5f, 0.75f);
    }
    else if (name == "general_rotation" || name == "free_scale" ||
             name == "scaled_small" || name == "scaled_large" ||
             name == "collinear" || name == "collinear_small" || name == "collinear_large" ||
             name == "near_collinear")
    {
        sample.rotation = Eigen::AngleAxisf(
            0.4f, Eigen::Vector3f(1.0f, 2.0f, 3.0f).normalized()).toRotationMatrix();
        sample.translation = Eigen::Vector3f(0.25f, -0.5f, 0.75f);
        if (name == "free_scale")
        {
            sample.fixed_scale = false;
            sample.scale = 2.5f;
        }
        if (name == "scaled_small" || name == "collinear_small") sample.geometry_scale = 1.0e-3f;
        if (name == "scaled_large" || name == "collinear_large") sample.geometry_scale = 1.0e3f;
        if (name == "collinear" || name == "collinear_small" || name == "collinear_large" ||
            name == "near_collinear")
        {
            sample.expect_rejection = true;
            sample.points2 << -1.0f, 0.0f, 1.0f,
                              0.0f, 0.0f, 0.0f,
                              4.0f, 4.0f, 4.0f;
            if (name == "near_collinear") sample.points2(1, 1) = 1.0e-6f;
        }
    }
    else if (name == "near_pi")
    {
        sample.rotation = Eigen::AngleAxisf(
            3.14159265358979323846f - 1.0e-4f, Eigen::Vector3f::UnitY()).toRotationMatrix();
        sample.translation = Eigen::Vector3f(0.25f, -0.5f, 0.75f);
    }
    else if (name == "pi")
    {
        sample.rotation << -1.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, -1.0f;
        sample.translation = Eigen::Vector3f(0.25f, -0.5f, 0.75f);
    }
    else if (name == "coincident" || name == "coincident_p1" || name == "coincident_p2")
    {
        sample.expect_rejection = true;
        sample.points1 = sample.points2;
        if (name != "coincident_p2")
            sample.points1.colwise() = Eigen::Vector3f(0.5f, -0.25f, 3.0f);
        if (name != "coincident_p1")
            sample.points2.colwise() = Eigen::Vector3f(0.0f, 0.0f, 4.0f);
        return sample;
    }
    else if (name == "runtime_sample")
    {
        // Exact float round-trip values from the Stage 2 runtime snapshot:
        // KF125/KF13, iteration 3, gdb.log line 71. P1 alone is collapsed.
        sample.expect_rejection = true;
        sample.points1 << -0.799052238f, -0.799052238f, -0.799052238f,
                          -0.291015834f, -0.291015834f, -0.291015834f,
                           1.19793129f, 1.19793129f, 1.19793129f;
        sample.points2 << 0.77207917f, 3.35247803f, 1.87498164f,
                          0.74389112f, 0.22841242f, 0.0977537334f,
                          3.15984201f, 3.82116652f, 2.53205013f;
        return sample;
    }
    else if (name == "nan_input" || name == "inf_input" ||
             name == "nan_input_p2" || name == "inf_input_p2")
    {
        sample.expect_rejection = true;
        sample.points1 = sample.points2;
        const float invalid = name.find("nan_") == 0 ?
            std::numeric_limits<float>::quiet_NaN() : std::numeric_limits<float>::infinity();
        if (name == "nan_input_p2" || name == "inf_input_p2") sample.points2(0, 0) = invalid;
        else sample.points1(0, 0) = invalid;
        return sample;
    }
    else if (name == "overflow_input")
    {
        // Individually finite coordinates overflow during float centroid sums.
        sample.expect_rejection = true;
        sample.points1 = sample.points2 * (std::numeric_limits<float>::max() / 8.0f);
        return sample;
    }
    else if (name == "overflow_covariance" || name == "underflow_covariance")
    {
        sample.expect_rejection = true;
        sample.points2 *= name == "overflow_covariance" ? 1.0e20f : 1.0e-25f;
        sample.points1 = sample.points2;
        return sample;
    }
    else if (name == "scale_denominator_overflow")
    {
        // Cross-covariance stays finite while the free-scale denominator
        // exceeds the float range. This exercises a rejection after rotation.
        sample.expect_rejection = true;
        sample.fixed_scale = false;
        sample.points1 = sample.points2 * 1.0e-10f;
        sample.points2 *= 1.0e20f;
        return sample;
    }
    else
    {
        throw std::invalid_argument("Unknown Sim3 case: " + name);
    }

    sample.points2 *= sample.geometry_scale;
    sample.translation *= sample.geometry_scale;
    sample.points1 = (sample.scale * sample.rotation * sample.points2).colwise() + sample.translation;
    return sample;
}

void check_transform(const TestSim3Solver& solver, const Sample& sample)
{
    require(solver.mT12i.allFinite() && solver.mT21i.allFinite(), "Transform is non-finite");
    require(solver.mR12i.allFinite() && solver.mt12i.allFinite() && std::isfinite(solver.ms12i),
            "Rotation, translation or scale is non-finite");
    const float scale_tolerance = sample.fixed_scale ? 1.0e-6f : 1.0e-5f;
    require(std::abs(solver.ms12i - sample.scale) < scale_tolerance, "Estimated scale changed");
    require((solver.mR12i - sample.rotation).cwiseAbs().maxCoeff() < 2.0e-5f,
            "Rotation does not match the known transform");
    require((solver.mt12i - sample.translation).norm() < 2.0e-4f * sample.geometry_scale,
            "Translation does not match the known transform");
    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
    expected.block<3, 3>(0, 0) = sample.scale * sample.rotation;
    expected.block<3, 1>(0, 3) = sample.translation;
    const float matrix_tolerance = 2.0e-4f * std::max(1.0f, sample.geometry_scale);
    require((solver.mT12i - expected).cwiseAbs().maxCoeff() < matrix_tolerance,
            "Forward transform does not match the known model");
    const Eigen::Matrix3f orthogonality = solver.mR12i.transpose() * solver.mR12i;
    require((orthogonality - Eigen::Matrix3f::Identity()).cwiseAbs().maxCoeff() < 1.0e-4f &&
            std::abs(solver.mR12i.determinant() - 1.0f) < 1.0e-4f,
            "Estimated rotation is not in SO3");
    const Eigen::Matrix3f predicted = (solver.ms12i * solver.mR12i * sample.points2).colwise() +
                                    solver.mt12i;
    require((predicted - sample.points1).cwiseAbs().maxCoeff() < 4.0e-4f * sample.geometry_scale,
            "Known correspondences are not reconstructed");
    const Eigen::Matrix4f inverse_product = solver.mT21i * solver.mT12i;
    const float inverse_tolerance = 1.0e-4f * std::max(1.0f, sample.geometry_scale);
    require((inverse_product - Eigen::Matrix4f::Identity()).cwiseAbs().maxCoeff() < inverse_tolerance,
            "Forward and inverse transforms disagree");
}

void check_ransac(const std::string& name)
{
    const bool mixed = name == "ransac_mixed_4arg" || name == "ransac_mixed_5arg";
    const bool with_convergence = name == "ransac_reject_5arg" || name == "ransac_mixed_5arg";
    require(mixed || name == "ransac_reject_4arg" || name == "ransac_reject_5arg",
            "Unknown RANSAC case");

    const FixturePoints positions2 = make_fixture_points();
    const Eigen::Matrix3f rotation = Eigen::AngleAxisf(0.08f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Vector3f translation(0.25f, -0.5f, 0.75f);
    FixturePoints positions1 = (rotation * positions2).colwise() + translation;
    if (mixed) positions1.col(1) = positions1.col(0); // Five good matches, one inconsistent match.
    else positions1.colwise() = Eigen::Vector3f(0.5f, -0.25f, 3.0f);

    Fixture fixture(positions1, positions2);
    TestSim3Solver solver(fixture.keyframe1.get(), fixture.keyframe2.get(), fixture.matches, true);
    solver.SetRansacParameters(0.99, 3, 9);
    std::cout << "SIM3_CASE name=" << name << " phase=iterate" << std::endl;

    Eigen::Matrix4f saved_best = Eigen::Matrix4f::Identity();
    const int calls = mixed ? 3 : 5;
    for (int step = 0; step < calls; ++step)
    {
        if (mixed)
        {
            // Force reject -> success -> reject using real sampled points,
            // independently of the C library's random sequence/permutations.
            solver.mvAllIndices = step == 1 ? std::vector<std::size_t>{2, 3, 4} :
                                                std::vector<std::size_t>{0, 1, 2};
        }

        const int before = solver.mnIterations;
        const int requested = mixed ? 1 : 2;
        bool no_more = true;
        bool converge = true;
        int inliers = 99;
        std::vector<bool> mask(fixture_points, true);
        Eigen::Matrix4f transform;
        if (with_convergence) transform = solver.iterate(requested, no_more, mask, inliers, converge);
        else transform = solver.iterate(requested, no_more, mask, inliers);

        require(transform.allFinite(), "RANSAC returned a non-finite model/placeholder");
        require(solver.mnIterations == std::min(before + requested, solver.mRansacMaxIts),
                "Rejected draws did not consume the iteration budget");
        if (mixed && step == 1)
        {
            require(inliers == 5 && std::count(mask.begin(), mask.end(), true) == 5 && !mask[1],
                    "Valid draw did not recover the five consistent matches");
            require(!with_convergence || converge, "Valid draw did not report convergence");
            require((solver.GetEstimatedRotation() - rotation).cwiseAbs().maxCoeff() < 2.0e-5f &&
                    (solver.GetEstimatedTranslation() - translation).norm() < 2.0e-4f &&
                    std::abs(solver.GetEstimatedScale() - 1.0f) < 1.0e-6f,
                    "RANSAC recovered the wrong known transform");
            saved_best = solver.GetEstimatedTransformation();
            require((transform - saved_best).cwiseAbs().maxCoeff() == 0.0f,
                    "RANSAC returned a different model from its best estimate");
        }
        else
        {
            require(inliers == 0 && std::count(mask.begin(), mask.end(), true) == 0,
                    "Rejected draw reused previous inliers");
            require(!with_convergence || !converge, "Rejected draw reported convergence");
            require((transform - Eigen::Matrix4f::Identity()).cwiseAbs().maxCoeff() == 0.0f,
                    "Failed search returned an uninitialized/stale model");
            require((solver.GetEstimatedTransformation() - saved_best).cwiseAbs().maxCoeff() == 0.0f &&
                    solver.mnBestInliers == (mixed && step == 2 ? 5 : 0),
                    "Rejected draw changed the stored best model");
        }
        require(no_more == (solver.mnIterations >= solver.mRansacMaxIts),
                "RANSAC exhaustion flag is inconsistent with the budget");
    }
    require(mixed || solver.mnIterations == solver.mRansacMaxIts,
            "All-rejected search did not terminate at its budget");
}

void check_rejection_after_success(TestSim3Solver& solver)
{
    const Eigen::Matrix3f rotation = solver.mR12i;
    const Eigen::Vector3f translation = solver.mt12i;
    const float scale = solver.ms12i;
    const Eigen::Matrix4f transform12 = solver.mT12i;
    const Eigen::Matrix4f transform21 = solver.mT21i;
    for (const char* name : {"runtime_sample", "nan_input", "collinear",
                             "overflow_covariance", "scale_denominator_overflow"})
    {
        Sample invalid = make_sample(name);
        require(!solver.ComputeSim3(invalid.points1, invalid.points2), "Invalid follow-up draw was accepted");
        require((solver.mR12i - rotation).cwiseAbs().maxCoeff() == 0.0f &&
                (solver.mt12i - translation).cwiseAbs().maxCoeff() == 0.0f && solver.ms12i == scale &&
                (solver.mT12i - transform12).cwiseAbs().maxCoeff() == 0.0f &&
                (solver.mT21i - transform21).cwiseAbs().maxCoeff() == 0.0f,
                "Rejected computation partially overwrote the last valid model");
    }
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: sim3_solver_test CASE (see CMakeLists.txt for registered cases)\n";
        return 2;
    }

    try
    {
        const std::string name = argv[1];
        std::cout.precision(std::numeric_limits<float>::max_digits10);
        std::cout << "SIM3_CASE name=" << name << " phase=fixture" << std::endl;
        std::string outcome;
        if (name.find("ransac_") == 0)
        {
            check_ransac(name);
            outcome = "ransac";
        }
        else
        {
            Sample sample = make_sample(name == "valid_then_invalid" ? "free_scale" : name);
            Fixture fixture;
            TestSim3Solver solver(fixture.keyframe1.get(), fixture.keyframe2.get(), fixture.matches,
                                  sample.fixed_scale);

            // Flush the supplied sample before the real solver call so unexpected
            // aborts remain distinguishable from construction/fixture failures.
            std::cout << "SIM3_CASE name=" << name << " phase=compute\n"
                      << "P1:\n" << sample.points1 << "\nP2:\n" << sample.points2 << std::endl;
            const bool accepted = solver.ComputeSim3(sample.points1, sample.points2);
            if (sample.expect_rejection)
            {
                require(!accepted, "Invalid sample was accepted");
                require(solver.mT12i.allFinite() && solver.mT21i.allFinite(),
                        "Rejected first draw left non-finite current state");
                outcome = "rejected";
            }
            else
            {
                require(accepted, "Valid sample was rejected");
                check_transform(solver, sample);
                if (name == "valid_then_invalid") check_rejection_after_success(solver);
                outcome = name == "valid_then_invalid" ? "state_preserved" : "accepted";
            }
        }
        std::cout << "SIM3_CHECK name=" << name << " result=PASS outcome=" << outcome << std::endl;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "SIM3_CASE name=" << argv[1] << " error=" << error.what() << std::endl;
        return 1;
    }
}
