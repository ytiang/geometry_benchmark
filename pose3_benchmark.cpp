#include <iostream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <benchmark/benchmark.h>
#include <chrono>

#include <random>
#include <Eigen/Dense>

class PoseGenerator : public ::benchmark::Fixture {
 public:
    void SetUp(const ::benchmark::State &state) {
        std::random_device rt;
        this->mt_ = std::mt19937(rt);
        this->dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
    }

    void TearDown(const ::benchmark::State &state) {

    }

    ~PoseGenerator() {

    }

 private:
    std::mt19937 mt_;
    std::uniform_real_distribution<double> dist_;
};

static void BM_gtsamPose3(benchmark::State &state) {
    int range = state.range(0);
    double yaw = M_PI * 2.0 / range;
    for (auto _:state) {
        auto start = std::chrono::high_resolution_clock::now();
        gtsam::Point3 p1(range / 3.0, range / 2.0, range / 1.0);
        gtsam::Point3 p2;
        auto rot = gtsam::Rot3::ypr(0.0, 0.0, yaw);
        gtsam::Pose3 transform(rot, gtsam::Point3());
        p2 = transform.inverse() * p1;
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                        end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
//    std::cout << "point: " << p1 << "result: " << p2 << "\n";
}
BENCHMARK(BM_gtsamPose3)
->RangeMultiplier(2)->Range(2, 40)->UseRealTime()->Threads(1)->UseRealTime();

static void BM_gtsamPose2(benchmark::State &state) {
    int range = state.range(0);
    gtsam::Point2 p1(range / 3.0, range / 2.0);
    gtsam::Point2 p2;
    double yaw = M_PI * 2.0 / range;
    for (auto _:state) {
        gtsam::Pose2 transform(0.0, 0.0, yaw);
        benchmark::DoNotOptimize(transform.inverse() * p1);
    }
//    std::cout << "point: " << p1 << "result: " << p2 << "\n";
}

BENCHMARK(BM_gtsamPose2)
->RangeMultiplier(2)->Range(2, 40)->UseRealTime()->Threads(2)->UseRealTime();

static void BM_selfPose2(benchmark::State &state) {
    int range = state.range(0);
    double x = range / 3.0, y = range / 2.0;
    double yaw = M_PI * 2.0 / range;
    for (auto _:state) {
        benchmark::DoNotOptimize(x * cos(yaw) + y * sin(yaw));
        benchmark::DoNotOptimize(-x * sin(yaw) + x * cos(yaw));
    }
//    std::cout << "point: " << p1 << "result: " << p2 << "\n";
}

BENCHMARK(BM_selfPose2)
->RangeMultiplier(2)->Range(2, 40)->UseRealTime()->Threads(2)->UseRealTime();

BENCHMARK_MAIN();
