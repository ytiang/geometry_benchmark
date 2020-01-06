//
// Created by yangt on 2019/12/2.
//

#include "yt_pose/pose3d.hpp"

/**************************************************************************** */
Pose3d::Pose3d()
    : PoseData(std::move(Quaternion(1.0, 0, 0, 0)),
               std::move(Point3::Zero())) {

}

/**************************************************************************** */
Pose3d::Pose3d(Pose3d &&pose)
    : PoseData(std::move(pose.quaterion()), std::move(pose.point())) {
}

/**************************************************************************** */
Pose3d::Pose3d(const Pose3d &pose)
    : PoseData(pose.quaterion(), pose.point()) {
}

/**************************************************************************** */
Pose3d::Pose3d(const Quaternion &q, const Point3 &t)
    : PoseData(q, t) {
}

/**************************************************************************** */
//Pose3d::Pose3d(Quaternion &&q, Point3 &&t)
//    : PoseData(std::move(q), std::move(t)) {
//
//}

/**************************************************************************** */
Point3 &Pose3d::point() {
    return std::get<1>(*this);
}

/**************************************************************************** */
Quaternion &Pose3d::quaterion() {
    return std::get<0>(*this);
}

/**************************************************************************** */
const Point3 &Pose3d::point() const {
    return std::get<1>(*this);
}

/**************************************************************************** */
const Quaternion &Pose3d::quaterion() const {
    return std::get<0>(*this);
}

/**************************************************************************** */
Pose3d::Rot3 Pose3d::rot() const {
    return std::move(quaterion().toRotationMatrix());
}

/**************************************************************************** */
Pose3d::Transform3 Pose3d::matrix() const {
    static const auto A14 = Eigen::RowVector4d(0, 0, 0, 1);
    Transform3 mat;
    mat << rot(), point(), A14;
    return mat;
}

/**************************************************************************** */
Pose3d Pose3d::inverse() const {
    Quaternion inv_q = quaterion().inverse();
    Point3 inv_t = -(inv_q * point());
    return Pose3d(inv_q, inv_t);
}


/**************************************************************************** */
double Pose3d::EuclidDis(const Pose3d &other) const {
    auto vec = this->point() - other.point();

    return vec.lpNorm<2>();
}

/**************************************************************************** */
double Pose3d::EuclidDis2D(const Pose3d &other) const {
    return sqrt(pow(this->point().x() - other.point().x(), 2)
                    + pow(this->point().y() - other.point().y(), 2));
}

/**************************************************************************** */
double Pose3d::angleDis(const Pose3d &other) const {
    double dot = quaterion().w() * other.quaterion().w() +
        quaterion().vec().dot(other.quaterion().vec());
    double angle = std::acos(dot);
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}



/**************************************************************************** */
Pose3d Pose3d::operator*(const Pose3d &other) const {
    Quaternion new_q = this->quaterion() * other.quaterion();
    Point3 new_t = this->quaterion() * other.point() + this->point();
    return Pose3d(new_q, new_t);
}

/**************************************************************************** */
Pose3d Pose3d::operator*(Pose3d &&other) const {
    other.quaterion() = this->quaterion() * other.quaterion();
    other.point() = this->quaterion() * other.point() + this->point();
    return std::move(other);
}

/**************************************************************************** */
Point3 Pose3d::operator*(const Point3 &other) const {
    return quaterion() * other + point();
}

/**************************************************************************** */
Pose3d &Pose3d::operator=(const Pose3d &other) {
    this->point() = other.point();
    this->quaterion() = other.quaterion();
    return (*this);
}