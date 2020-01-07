//
// Created by yangt on 2019/12/2.
//

#ifndef TERRAIN_UTILIS_TOOLS_HPP
#define TERRAIN_UTILIS_TOOLS_HPP

#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
// #include <glog/logging.h>
#include <chrono>

#include "pose3d.hpp"

geometry_msgs::Quaternion toQuaternionMsg(const Quaternion &rotation);

geometry_msgs::Point toPointMsg(const Point3 &point);

geometry_msgs::Pose toPoseMsg(const Pose3d &pose);

Pose3d msgToPose(const geometry_msgs::Pose &msg);

Quaternion eulerToQuaternion(Real roll, Real pitch, Real yaw);

Real quaternionToYaw(const Quaternion &orien);

Point3 quternionToEuler(const Quaternion &orien);

double positiveAngle(double angle);

double normalizeAngle(double angle);

std::chrono::time_point<std::chrono::steady_clock> now();

double duration(
    const std::chrono::time_point<std::chrono::steady_clock> &start,
    const std::chrono::time_point<std::chrono::steady_clock> &end);

template<class Value>
Value getParam(const YAML::Node &node,
               const std::string &name,
               Value default_val) {
    Value v;
    try {
        v = node[name].as<Value>();
        std::cout << "--Found parameter: " << name << ",\tvalue: " << v << "\n";
    } catch (std::exception &e) {
        v = default_val;
        std::cout << "--Cannot find parameter: " << name
                  << ",\tassigning  default: " << v << "\n";
    }
    return v;
}

#endif //TERRAIN_UTILIS_TOOLS_HPP
