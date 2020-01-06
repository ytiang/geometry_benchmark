//
// Created by yangt on 2019/12/2.
//

#ifndef TERRAIN_UTILIS_POSE3D_HPP
#define TERRAIN_UTILIS_POSE3D_HPP

#include "common_definition.hpp"

#include <tuple>

class Pose3d : public std::tuple<Quaternion, Point3> {
 public:
    typedef std::tuple<Quaternion, Point3> PoseData;
    typedef Matrix<3> Rot3;
    typedef Matrix<4> Transform3;

    using PoseData::PoseData;

    Pose3d();

    Pose3d(Pose3d &&pose);

    Pose3d(const Pose3d &pose);

    Pose3d(const Quaternion &q, const Point3 &t);

//    Pose3d(Quaternion &&q, Point3 &&t);

    Point3 &point();

    Quaternion &quaterion();

    const Point3 &point() const;

    const Quaternion &quaterion() const;

    Rot3 rot() const;

    Transform3 matrix() const;

    Pose3d inverse() const;

    double EuclidDis(const Pose3d &other) const;

    double EuclidDis2D(const Pose3d &other) const;

    double angleDis(const Pose3d &other) const;

    Pose3d operator*(const Pose3d &other) const;

    Pose3d operator*(Pose3d &&other) const;

    Point3 operator*(const Point3 &other) const;

    Pose3d &operator=(const Pose3d &other);

};

#endif //TERRAIN_UTILIS_POSE3D_HPP
