#include "gemini336_orbslam3/logging.hpp"
#include <GBADiagnosticStatistics.h>
#include <ImuTypes.h>
#include <Thirdparty/g2o/g2o/core/block_solver.h>
#include <Thirdparty/g2o/g2o/core/robust_kernel_impl.h>
#include <Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace
{
void Require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

// Constant residual fixtures exercise real edge categories/kernels without
// cameras or KeyFrames. The fixture optimizer owns none of these stack edges.
template<class Edge>
class ConstantEdge : public Edge
{
public:
  ConstantEdge() { this->_information.setIdentity(); this->_error.setOnes(); }
  void computeError() override {}
  void SetRaw(double value) { this->_error.setZero(); this->_error[0] = std::sqrt(value); }
};

class ConstantInertial : public ORB_SLAM3::EdgeInertial
{
public:
  explicit ConstantInertial(ORB_SLAM3::IMU::Preintegrated* preint) : EdgeInertial(preint)
  { _information.setIdentity(); _error.setOnes(); }
  void computeError() override {}
};

class FixtureOptimizer : public g2o::SparseOptimizer
{
public:
  void SetEdges(const EdgeContainer& edges) { _activeEdges = edges; }
};

class ScalarVertex : public g2o::BaseVertex<1, double>
{
public:
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
  void setToOriginImpl() override { _estimate = 0; }
  void oplusImpl(const double* update) override { _estimate += update[0]; }
};

class ScalarEdge : public g2o::BaseUnaryEdge<1, double, ScalarVertex>
{
public:
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
  void computeError() override
  { _error[0] = static_cast<const ScalarVertex*>(_vertices[0])->estimate() - _measurement; }
  void linearizeOplus() override { _jacobianOplusXi[0] = 1; }
};

void TestObjective()
{
  using namespace ORB_SLAM3;
  IMU::Calib calibration;
  calibration.Cov.setIdentity();
  calibration.CovWalk.setIdentity();
  IMU::Preintegrated preint(IMU::Bias(), calibration);
  preint.C.setIdentity();  // Quiescent synthetic fixture, not a live map.
  ConstantEdge<EdgeMono> mono;
  ConstantEdge<EdgeStereo> stereo;
  ConstantInertial imu(&preint);
  ConstantEdge<EdgeGyroRW> gyro;
  ConstantEdge<EdgeAccRW> accel;
  ScalarEdge other;
  ScalarVertex vertex;
  vertex.setEstimate(2);
  other.setVertex(0, &vertex);
  other.setMeasurement(0);
  other.setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  mono.SetRaw(25);
  auto* kernel = new g2o::RobustKernelHuber;
  kernel->setDelta(1);
  mono.setRobustKernel(kernel);
  FixtureOptimizer optimizer;
  optimizer.SetEdges({&mono, &stereo, &imu, &gyro, &accel, &other});
  optimizer.computeActiveErrors();
  const auto totals = GBADiagnostics::SummarizeObjective(optimizer);
  for (const auto& category : totals.categories) Require(category.count == 1, "Wrong category count");
  Require(std::abs(totals.categories[0].raw - 25) < 1e-10, "Wrong raw mono objective");
  Require(std::abs(totals.categories[0].robust - 9) < 1e-10, "Huber rho[0] not used");
  Require(GBADiagnostics::ObjectiveSumMatches(totals.robust_sum, optimizer.activeRobustChi2()),
          "Category sum does not reconstruct g2o objective");
  mono.SetRaw(std::numeric_limits<double>::quiet_NaN());
  const auto invalid = GBADiagnostics::SummarizeObjective(optimizer);
  Require(invalid.categories[0].nonfinite == 1, "Nonfinite edge not counted");
  Require(!GBADiagnostics::ObjectiveSumMatches(invalid.robust_sum, optimizer.activeRobustChi2()),
          "Nonfinite objective incorrectly matched");
}

double SolveScalar(bool observe, bool throwFromObserver, std::size_t& trials)
{
  g2o::SparseOptimizer optimizer;
  using Block = g2o::BlockSolverX;
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(
    new Block(new g2o::LinearSolverEigen<Block::PoseMatrixType>));
  solver->setUserLambdaInit(1e-5);
  bool normalTrials = true;
  if (observe) {
    solver->setTrialDiagnosticCallback([&](const auto& trial) {
      ++trials;
      normalTrials = normalTrials && !ORB_SLAM3::GBADiagnostics::TrialIsAnomalous(trial);
      if (throwFromObserver) throw std::runtime_error("Injected observer failure");
    });
  }
  optimizer.setAlgorithm(solver);
  auto* vertex = new ScalarVertex;
  vertex->setId(0);
  vertex->setEstimate(0);
  optimizer.addVertex(vertex);
  auto* edge = new ScalarEdge;
  edge->setVertex(0, vertex);
  edge->setMeasurement(3);
  edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  optimizer.addEdge(edge);
  Require(optimizer.initializeOptimization(), "Synthetic graph initialization failed");
  Require(optimizer.optimize(7) > 0, "Synthetic LM solve failed");
  Require(normalTrials, "Unexpected scalar trial anomaly");
  return vertex->estimate();
}

void TestTrials()
{
  using Trial = g2o::OptimizationAlgorithmLevenberg::TrialDiagnostic;
  std::size_t disabled = 0, enabled = 0, throwing = 0;
  const double baseline = SolveScalar(false, false, disabled);
  Require(std::abs(baseline - 3) < 1e-8, "Synthetic solver did not reach target");
  Require(std::abs(SolveScalar(true, false, enabled) - baseline) < 1e-12, "Observer changed solution");
  Require(std::abs(SolveScalar(true, true, throwing) - baseline) < 1e-12, "Observer exception changed solution");
  Require(disabled == 0 && enabled > 0 && throwing > 0, "Callback gating failed");
  Trial trial{0, 0, 10, 5, 5, 1, 5, 1e-5, 1e-6, true, true};
  Require(!ORB_SLAM3::GBADiagnostics::TrialIsAnomalous(trial), "Normal trial flagged");
  trial.evaluatedChi = 20;
  Require(ORB_SLAM3::GBADiagnostics::TrialIsAnomalous(trial), "Accepted cost increase missed");
  trial.evaluatedChi = 5; trial.scale = 0;
  Require(ORB_SLAM3::GBADiagnostics::TrialIsAnomalous(trial), "Nonpositive scale missed");
  trial.scale = 1; trial.rho = std::numeric_limits<double>::infinity();
  Require(ORB_SLAM3::GBADiagnostics::TrialIsAnomalous(trial), "Nonfinite rho missed");
  trial.rho = 5; trial.linearSolveOK = false;
  Require(ORB_SLAM3::GBADiagnostics::TrialIsAnomalous(trial), "Linear failure missed");
}

void TestPreintegrationIsolation()
{
  using namespace ORB_SLAM3;
  IMU::Calib calibration;
  calibration.Cov.setIdentity();
  calibration.CovWalk.setIdentity();
  IMU::Preintegrated source(IMU::Bias(), calibration), previous(IMU::Bias(), calibration);
  const Eigen::Vector3f acceleration(1, 2, 9.81f), angular(.01f, -.02f, .03f);
  source.IntegrateNewMeasurement(acceleration, angular, .01f);
  previous.IntegrateNewMeasurement(acceleration, angular, .02f);
  source.C.setIdentity();  // Quiescent fixture ensures invertible edge information.
  const std::unique_ptr<IMU::Preintegrated> frozen = std::make_unique<IMU::Preintegrated>(&source);
  const auto snapshot = frozen->GetDiagnosticSnapshot();
  const Eigen::Matrix<float, 15, 15> covariance = frozen->C;
  const Eigen::Matrix3f rotation = frozen->GetDeltaRotation(IMU::Bias());
  const Eigen::Vector3f velocity = frozen->GetDeltaVelocity(IMU::Bias());
  const Eigen::Vector3f position = frozen->GetDeltaPosition(IMU::Bias());
  Require(snapshot.object != source.GetDiagnosticSnapshot().object, "Snapshot shares source identity");

  // Real inertial residuals, with fixed synthetic vertices, reproduce the
  // cached-edge/live-source mismatch without launching any SLAM workers.
  ImuCamPose pose;
  pose.Rwb.setIdentity(); pose.Rwb0.setIdentity(); pose.DR.setIdentity();
  pose.twb.setZero(); pose.bf = 0; pose.its = 0;
  VertexPose firstPose, secondPose;
  firstPose.setEstimate(pose); secondPose.setEstimate(pose);
  VertexVelocity firstVelocity, secondVelocity;
  firstVelocity.setEstimate(Eigen::Vector3d::Zero());
  secondVelocity.setEstimate(Eigen::Vector3d::Zero());
  VertexGyroBias gyro;
  VertexAccBias accel;
  // Nonzero bias corrections also exercise the copied preintegration Jacobians.
  gyro.setEstimate(Eigen::Vector3d(.002, -.001, .003));
  accel.setEstimate(Eigen::Vector3d(.1, -.2, .05));
  EdgeInertial frozenEdge(frozen.get()), liveEdge(&source);
  for (EdgeInertial* edge : {&frozenEdge, &liveEdge}) {
    edge->setVertex(0, &firstPose); edge->setVertex(1, &firstVelocity);
    edge->setVertex(2, &gyro); edge->setVertex(3, &accel);
    edge->setVertex(4, &secondPose); edge->setVertex(5, &secondVelocity);
    edge->computeError();
  }
  const Vector9d error = frozenEdge.error();
  Require(error.allFinite() && (liveEdge.error() - error).norm() < 1e-12,
          "Snapshot changed initial inertial residual");
  const Matrix9d information = frozenEdge.information();
  source.MergePrevious(&previous);
  liveEdge.computeError();
  Require((liveEdge.error() - error).norm() > 1e-6, "Fixture did not reproduce live-source mutation");
  source.SetNewBias(IMU::Bias(.1f, 0, 0, 0, 0, 0));
  source.Reintegrate();
  source.CopyFrom(&previous);
  source.Initialize(IMU::Bias());
  source.IntegrateNewMeasurement(acceleration, angular, .03f);
  frozenEdge.computeError();
  const auto after = frozen->GetDiagnosticSnapshot();
  Require(after.revision == snapshot.revision && after.measurements == snapshot.measurements && after.dt == snapshot.dt,
          "Source mutation changed snapshot metadata");
  Require((frozen->C - covariance).norm() == 0, "Source mutation changed IMU/bias covariance snapshot");
  Require((frozen->GetDeltaRotation(IMU::Bias()) - rotation).norm() == 0 &&
          (frozen->GetDeltaVelocity(IMU::Bias()) - velocity).norm() == 0 &&
          (frozen->GetDeltaPosition(IMU::Bias()) - position).norm() == 0,
          "Source mutation changed snapshot deltas");
  Require(frozenEdge.dt == snapshot.dt && (frozenEdge.information() - information).norm() == 0 &&
          (frozenEdge.error() - error).norm() < 1e-12,
          "Source mutation changed frozen edge time/information/residual");
}

void TestMutation(const std::shared_ptr<spdlog::logger>& logger)
{
  using namespace ORB_SLAM3;
  IMU::Calib calibration;
  calibration.Cov.setIdentity(); calibration.CovWalk.setIdentity();
  IMU::Preintegrated current(IMU::Bias(), calibration), previous(IMU::Bias(), calibration);
  const Eigen::Vector3f acceleration(0, 0, 9.81f), angular = Eigen::Vector3f::Zero();
  current.IntegrateNewMeasurement(acceleration, angular, .01f);
  previous.IntegrateNewMeasurement(acceleration, angular, .02f);
  auto context = std::make_shared<IMU::Preintegrated::DiagnosticContext>();
  context->logger = logger; context->map = 0; context->loop_kf = 619;
  context->generation = 1; context->kf = 12; context->previous_kf = 10;
  const auto before = current.WatchForGBA(context);
  const auto originalDelta = current.GetDeltaPosition(IMU::Bias());
  current.SetNewBias(IMU::Bias(.1f, 0, 0, 0, 0, 0));
  const auto bias = current.GetDiagnosticSnapshot();
  Require(bias.revision == before.revision && bias.bias_revision > before.bias_revision,
          "Updated bias incorrectly counted as explicit-bias residual mutation");
  Require((originalDelta - current.GetDeltaPosition(IMU::Bias())).norm() == 0, "Explicit-bias delta changed");
  current.MergePrevious(&previous);
  const auto merged = current.GetDiagnosticSnapshot();
  Require(merged.object == before.object && merged.revision > before.revision && merged.measurements == 2,
          "Merge identity/revision/count not captured");
  Require(std::abs(merged.dt - .03f) < 1e-6, "Merge dT incorrect");
  current.Reintegrate();
  const auto reintegrated = current.GetDiagnosticSnapshot();
  Require(reintegrated.revision > merged.revision && reintegrated.measurements == 2, "Reintegration not tracked");
  current.CopyFrom(&previous);
  Require(current.GetDiagnosticSnapshot().revision > reintegrated.revision, "CopyFrom not tracked");
  current.Initialize(IMU::Bias());
  Require(current.GetDiagnosticSnapshot().measurements == 0, "Initialize not tracked");
  context->active.store(false);
  current.IntegrateNewMeasurement(acceleration, angular, .01f);
  IMU::Preintegrated copy(&current);
  Require(copy.GetDiagnosticSnapshot().object != before.object, "Copy inherited source identity");

  // Exercise metadata reads concurrent with a formerly unlocked integration
  // entrypoint. No sensors, ROS nodes, or SLAM workers are involved.
  IMU::Preintegrated concurrent(IMU::Bias(), calibration);
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (int i = 0; i < 500; ++i) concurrent.IntegrateNewMeasurement(acceleration, angular, .001f);
    done.store(true);
  });
  bool coherent = true;
  while (!done.load()) {
    const auto snapshot = concurrent.GetDiagnosticSnapshot();
    coherent = coherent && snapshot.revision == snapshot.measurements + 1 &&
      std::abs(snapshot.dt - snapshot.measurements * .001) < 1e-5;
  }
  writer.join();
  Require(coherent && concurrent.GetDiagnosticSnapshot().measurements == 500, "Concurrent snapshot incoherent");
}
}  // namespace

int main()
{
  try {
    TestObjective();
    TestTrials();
    TestPreintegrationIsolation();
    gemini336_orbslam3::LoggingOptions options;
    options.directory = std::filesystem::temp_directory_path() / ("gba_diagnostics_test_" + std::to_string(getpid()));
    options.level = "debug";
    gemini336_orbslam3::LoggingSession session(options);
    TestMutation(session.GetLogger("loop_closing"));
    session.finish();
    std::ifstream input(session.directory() / "slam.log");
    const std::string log{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    Require(log.find("EVENT=IMU_PREINT_MERGE") != std::string::npos, "Missing merge event");
    for (const char* path : {"PATH=REINTEGRATE", "PATH=COPY_FROM", "PATH=INITIALIZE"})
      Require(log.find(path) != std::string::npos, "Missing mutation path event");
    Require(log.find("PATH=INTEGRATE") == std::string::npos, "Inactive observer still logged");
    Require(session.dropped_messages() == 0, "Unit test logs dropped");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
