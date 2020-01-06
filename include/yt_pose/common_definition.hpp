//
// Created by yangt on 2019/12/2.
//

#ifndef TERRAIN_UTILIS_COMMON_DEFINITION_HPP
#define TERRAIN_UTILIS_COMMON_DEFINITION_HPP

#include<Eigen/Dense>

typedef double Real;

template<int Dim>
using Matrix=Eigen::Matrix<Real, Dim, Dim>;

template<int Dim>
using Vector=Eigen::Matrix<Real, Dim, 1>;

typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> VectorX;
typedef Vector<3> Point3;
typedef Vector<2> Point2;
typedef Eigen::Quaternion<Real> Quaternion;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IdMatrixX;

#endif //TERRAIN_UTILIS_COMMON_DEFINITION_HPP
