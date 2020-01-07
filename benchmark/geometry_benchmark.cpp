//
// Created by yangt on 2020/1/4.
//
#include <benchmark/benchmark.h>

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include "google_transform/rigid_transform.h"
#include "yt_pose/pose3d.hpp"

#include <Eigen/Dense>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <wave/geometry/geometry.hpp>

#include <iostream>
#include <random>
#include <chrono>

namespace ct = cartographer::transform;
namespace mp = mrpt::poses;

struct State2 {
    double x, y, yaw;
};
struct State3 {
    double x, y, z;
    double roll, pitch, yaw;
};

State2 ManualTrans2d(const State2 &ref, const State2 &input) {
    State2 output;
    output.x = input.x * cos(ref.yaw) - input.y * sin(ref.yaw) + ref.x;
    output.y = input.x * sin(ref.yaw) + input.y * cos(ref.yaw) + ref.y;
    output.yaw = input.yaw + ref.yaw;
    return output;
}

Eigen::Quaterniond eulerToQuaterion(double r, double p, double y) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
    return q;
}

class PoseGenerator : public ::benchmark::Fixture {
 public:
    void SetUp(const ::benchmark::State &state) {
        std::random_device rt;
        std::mt19937 mt(rt());
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        const double min_value = -100;
        const double len = 200;

        // initialize 2d:
        input2d.x = min_value + dist(mt) * len;
        input2d.y = min_value + dist(mt) * len;
        input2d.yaw = -M_PI + dist(mt) * 2 * M_PI;

        // initialize 3d
        input3d.x = min_value + dist(mt) * len;
        input3d.y = min_value + dist(mt) * len;
        input3d.z = min_value + dist(mt) * len;
        input3d.roll = -M_PI + dist(mt) * 2 * M_PI;
        input3d.pitch = -M_PI + dist(mt) * 2 * M_PI;
        input3d.yaw = -M_PI + dist(mt) * 2 * M_PI;

        State2 trans2;
        State3 trans3;
        for (int i = 0; i < 100; ++i) {
            trans2.x = min_value + dist(mt) * len;
            trans2.y = min_value + dist(mt) * len;
            trans2.yaw = -M_PI + dist(mt) * 2 * M_PI;

            this->trans2d_set.emplace_back(trans2);

            trans3.x = min_value + dist(mt) * len;
            trans3.y = min_value + dist(mt) * len;
            trans3.z = min_value + dist(mt) * len;
            trans3.roll = -M_PI + dist(mt) * 2 * M_PI;
            trans3.pitch = -M_PI + dist(mt) * 2 * M_PI;
            trans3.yaw = -M_PI + dist(mt) * 2 * M_PI;

            this->trans3d_set.emplace_back(trans3);
        }
    }

    void TearDown(const ::benchmark::State &state) {

    }

    ~PoseGenerator() {

    }

    State2 input2d;
    State3 input3d;

    std::vector<State2> trans2d_set;
    std::vector<State3> trans3d_set;
};

BENCHMARK_DEFINE_F(PoseGenerator, Manual2dTrans)(benchmark::State &st) {
    State2 output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
//            output = ManualTrans2d(trans, input2d);
            benchmark::DoNotOptimize(output = ManualTrans2d(trans, input2d));
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, Manual2dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   GtsamPose2dConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            const gtsam::Pose2 ref(trans.x, trans.y, trans.yaw);
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, GtsamPose2dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, GtsamPose2dTrans)(benchmark::State &st) {
    const gtsam::Pose2 input(input2d.x, input2d.y, input2d.yaw);
    gtsam::Pose2 output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            const gtsam::Pose2 ref(trans.x, trans.y, trans.yaw);
//            output = ref * input;
            benchmark::DoNotOptimize(output = ref * input);

        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, GtsamPose2dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   Cartographer2dConstruct)(benchmark::State &st) {

    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            ct::Rigid2d ref(Eigen::Vector2d(trans.x, trans.y), trans.yaw);
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, Cartographer2dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, Cartographer2dTrans)(benchmark::State &st) {
    ct::Rigid2d input(Eigen::Vector2d(input2d.x, input2d.y), input2d.yaw);
    ct::Rigid2d output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            ct::Rigid2d ref(Eigen::Vector2d(trans.x, trans.y), trans.yaw);
//            output = ref * input;
            benchmark::DoNotOptimize(output = ref * input);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, Cartographer2dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, MrptPose2dConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            mp::CPose2D ref(trans.x, trans.y, trans.yaw);
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, MrptPose2dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, MrptPose2dTrans)(benchmark::State &st) {
    const mp::CPose2D input(input2d.x, input2d.y, input2d.yaw);
    mp::CPose2D output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            mp::CPose2D ref(trans.x, trans.y, trans.yaw);
//            output = ref + input;
            benchmark::DoNotOptimize(output = ref + input);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, MrptPose2dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   GtsamPose3dConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans2d_set) {
            const gtsam::Pose3 ref
                (gtsam::Rot3::ypr(input3d.yaw, input3d.pitch, input3d.roll),
                 gtsam::Point3(input3d.x, input3d.y, input3d.z));
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, GtsamPose3dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, GtsamPose3dTrans)(benchmark::State &st) {
    const gtsam::Pose3 input
        (gtsam::Rot3::ypr(input3d.yaw, input3d.pitch, input3d.roll),
         gtsam::Point3(input3d.x, input3d.y, input3d.z));
    gtsam::Pose3 output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            const gtsam::Pose3 ref
                (gtsam::Rot3::ypr(input3d.yaw, input3d.pitch, input3d.roll),
                 gtsam::Point3(input3d.x, input3d.y, input3d.z));
//            output = ref * input;
            benchmark::DoNotOptimize(output = ref * input);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, GtsamPose3dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   Cartographer3dConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            const ct::Rigid3d
                ref(Eigen::Vector3d(trans.x, trans.y, trans.z),
                    eulerToQuaterion(trans.roll,
                                     trans.pitch,
                                     trans.yaw));
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, Cartographer3dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, Cartographer3dTrans)(benchmark::State &st) {
    const ct::Rigid3d input(Eigen::Vector3d(input3d.x, input3d.y, input3d.z),
                            eulerToQuaterion(input3d.roll,
                                             input3d.pitch,
                                             input3d.yaw));
    ct::Rigid3d output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            const ct::Rigid3d
                ref(Eigen::Vector3d(trans.x, trans.y, trans.z),
                    eulerToQuaterion(trans.roll,
                                     trans.pitch,
                                     trans.yaw));
//            output = ref * input;
            benchmark::DoNotOptimize(output = ref * input);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, Cartographer3dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, MrptPose3dConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            mp::CPose3D ref
                (trans.x, trans.y, trans.z, trans.yaw, trans.pitch, trans.roll);
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, MrptPose3dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, MrptPose3dTrans)(benchmark::State &st) {
    const mp::CPose3D input(input3d.x,
                            input3d.y,
                            input3d.z,
                            input3d.yaw,
                            input3d.pitch,
                            input3d.roll);
    mp::CPose3D output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {

            mp::CPose3D ref(trans.x,
                            trans.y,
                            trans.z,
                            trans.yaw,
                            trans.pitch,
                            trans.roll);
//            output = ref + input;
            benchmark::DoNotOptimize(output = ref + input);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, MrptPose3dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   YTPose3dConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            const Pose3d ref(eulerToQuaterion(trans.roll,
                                              trans.pitch,
                                              trans.yaw),
                             Eigen::Vector3d(trans.x, trans.y, trans.z));
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, YTPose3dConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator, YTPose3dTrans)(benchmark::State &st) {
    const Pose3d input(eulerToQuaterion(input3d.roll,
                                        input3d.pitch,
                                        input3d.yaw),
                       Eigen::Vector3d(input3d.x, input3d.y, input3d.z));
    Pose3d output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            const Pose3d ref(eulerToQuaterion(trans.roll,
                                              trans.pitch,
                                              trans.yaw),
                             Eigen::Vector3d(trans.x, trans.y, trans.z));
//            output = ref * input;
            benchmark::DoNotOptimize(output = ref * input);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, YTPose3dTrans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   WaveGeometryConstruct)(benchmark::State &st) {
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {

            const wave::RigidTransformMd ref(eulerToQuaterion(trans.roll,
                                                              trans.pitch,
                                                              trans.yaw),
                                             Eigen::Vector3d(trans.x,
                                                             trans.y,
                                                             trans.z));
            benchmark::ClobberMemory();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, WaveGeometryConstruct)
->Iterations(1000000)->UseManualTime();

BENCHMARK_DEFINE_F(PoseGenerator,
                   WaveGeometry3dTRans)(benchmark::State &st) {
    const wave::RigidTransformMd input(eulerToQuaterion(input3d.roll,
                                                        input3d.pitch,
                                                        input3d.yaw),
                                       Eigen::Vector3d(input3d.x,
                                                       input3d.y,
                                                       input3d.z));
    wave::RigidTransformMd output;
    for (auto _:st) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (const auto &trans:trans3d_set) {
            const wave::RigidTransformMd ref(eulerToQuaterion(trans.roll,
                                                              trans.pitch,
                                                              trans.yaw),
                                             Eigen::Vector3d(trans.x,
                                                             trans.y,
                                                             trans.z));
//            output = ref * input;
            benchmark::DoNotOptimize(output = ref * input);

        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        st.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK_REGISTER_F(PoseGenerator, WaveGeometry3dTRans)
->Iterations(1000000)->UseManualTime();

BENCHMARK_MAIN();