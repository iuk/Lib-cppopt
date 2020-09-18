// This file is part of cppopt, a lightweight C++ library
// for numerical optimization
//
// Copyright (C) 2014 Christoph Heindl <christoph.heindl@gmail.com>
//
// This Source Code Form is subject to the terms of the BSD 3 license.
// If a copy of the MPL was not distributed with this file, You can obtain
// one at http://opensource.org/licenses/BSD-3-Clause.

#include <cppopt/newton_raphson.h>
#include <cppopt/gauss_newton.h>
#include <iomanip>
#include <iostream>

/** This example finds a local extremum of a third order univariate polynomial using the Newton-Raphson algorithm.
 *
 *  The function to be optimized is given by 
 *
 *      f(x) = 3x^3 - 10x^2 - 56x + 5
 *
 *  In order to use Newton-Raphson for optimization the first and second order derivates are
 *  required, which are given by
 *
 *      df/dx   = 9x^2 - 20x - 56
 *      df/dxdx = 18x - 20
 *
 *  Note that the Newton-Raphson method is usually used for root finding where one only requires the function and
 *  the first order derivates. Since stationary points (extremum points, saddle points) are defined by having a gradient
 *  of zero (i.e root finding of the first order derivative)
 *
 *      df/dx = 0
 *
 *  we will simply pass the first and second order derivative into Newton-Raphson.
 */

// int main() {
//   // Define the first order derivative
//   cppopt::F df = [](const cppopt::Matrix &x) -> cppopt::Matrix {
//     cppopt::Matrix d(1, 1);

//     d(0) = 9.f * powf(x(0), 2) - 20.f * x(0) - 56.f;

//     return d;
//   };

//   // Define the second order derivative
//   cppopt::F ddf = [](const cppopt::Matrix &x) -> cppopt::Matrix {
//     cppopt::Matrix d(1, 1);

//     d(0) = 18 * x(0) - 20.f;

//     return d;
//   };

//   // Create a start solution. Note that this method does not necessarily find a minimum. Depending on its start value,
//   // it will convert to a minimum, maximum or saddle point. For example, try to use zero as start solution and the algorithm
//   // will find a maximum instead of a minimum.
//   cppopt::Matrix x(1, 1);
//   x(0) = 2.f;

//   // Iterate while norm of the first order derivative is greater than some predefined threshold.
//   cppopt::ResultInfo ri = cppopt::SUCCESS;
//   while (ri == cppopt::SUCCESS && df(x).norm() > 0.001f) {
//     ri = cppopt::newtonRaphson(df, ddf, x);
//     std::cout
//         << std::fixed << std::setw(3)
//         << "Parameters: " << x.transpose()
//         << " Error: " << df(x).norm() << std::endl;
//   }

//   std::cout << "Found a " << (ddf(x)(0) < 0.f ? "Maximum" : "Minimum") << std::endl;

//   assert(fabs(x(0) - cppopt::Scalar(3.841)) < cppopt::Scalar(0.001));

//   return 0;
// }

#include "lk_types.h"
#ifndef M_PIPI
#define M_PIPI 6.28318530718
#endif
#ifndef M_PIPIPI
#define M_PIPIPI 9.42477796077
#endif
#ifndef dutorad
#define dutorad(X) ((X) / 180.0 * M_PI)
#endif
#ifndef radtodu
#define radtodu(X) ((X) / M_PI * 180.0)
#endif

#ifndef modulo
#define modulo(X, Y) (((X % Y) + Y) % Y)
#endif

double angleLimit(double angle_in) {  // +- pi
  angle_in = fmod(angle_in + M_PI, M_PIPI);
  if (angle_in < 0)
    angle_in += M_PIPI;
  return angle_in - M_PI;
}

lk::Point3d AB_To_BA(lk::Point3d Ab) {
  lk::Point3d Ba;
  double cosAbz = cos(Ab.z);
  double sinAbz = sin(Ab.z);
  Ba.x          = -Ab.x * cosAbz - Ab.y * sinAbz;
  Ba.y          = Ab.x * sinAbz - Ab.y * cosAbz;
  Ba.z          = angleLimit(-Ab.z);
  return Ba;
}

lk::Point3d AB_BC_To_AC(lk::Point3d AB, lk::Point3d BC) {
  lk::Point3d AC;
  double cosABz = cos(AB.z);
  double sinABz = sin(AB.z);
  AC.x          = AB.x + BC.x * cosABz - BC.y * sinABz;
  AC.y          = AB.y + BC.y * cosABz + BC.x * sinABz;
  AC.z          = angleLimit(AB.z + BC.z);
  return AC;
}

int main() {
  lk::Point3d robot1_robot2(0.2, 0, 0);
  lk::Point3d robot_lidar(0.11, 0, dutorad(92));
  lk::Point3d lidar1_robot1 = AB_To_BA(robot_lidar);
  lk::Point3d lidar1_robot2 = AB_BC_To_AC(lidar1_robot1, robot1_robot2);
  lk::Point3d lidar1_lidar2 = AB_BC_To_AC(lidar1_robot2, robot_lidar);

  lidar1_lidar2.x += 0.01;
  lidar1_lidar2.y -= 0.01;
  lidar1_lidar2.z += dutorad(2);

  double Robot1Robot2_x = robot1_robot2.x;
  double Robot1Robot2_y = robot1_robot2.y;
  double Robot1Robot2_z = robot1_robot2.z;
  double Lidar1Lidar2_x = lidar1_lidar2.x;
  double Lidar1Lidar2_y = lidar1_lidar2.y;
  double Lidar1Lidar2_z = lidar1_lidar2.z;

  cppopt::F f = [&](const cppopt::Matrix &x) -> cppopt::Matrix {
    double RobotLidar_z = x(0);

    cppopt::Matrix d(1, 1);

    // d(0) = Robot1Robot2_x * (Lidar1Lidar2_y + (0.11 * cos(Lidar1Lidar2_z) * sin(RobotLidar_z)) - (0.11 * cos(RobotLidar_z) * sin(Lidar1Lidar2_z))) +
    //        Robot1Robot2_y * ((0.11 * cos(Lidar1Lidar2_z) * cos(RobotLidar_z)) - Lidar1Lidar2_x + (0.11 * sin(Lidar1Lidar2_z) * sin(RobotLidar_z)));

    d(0) = RobotLidar_z -
           atan2((Lidar1Lidar2_y + (11 * cos(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100 -
                  (11 * cos(RobotLidar_z) * sin(Lidar1Lidar2_z)) / 100),
                 ((11 * cos(Lidar1Lidar2_z) * cos(RobotLidar_z)) / 100 -
                  Lidar1Lidar2_x + (11 * sin(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100)) -
           atan2(Robot1Robot2_y, Robot1Robot2_x);

    return d;
  };

  cppopt::F df = [&](const cppopt::Matrix &x) -> cppopt::Matrix {
    double RobotLidar_z = x(0);

    cppopt::Matrix d(1, 1);

    // d(0) = Robot1Robot2_x * ((0.11 * cos(Lidar1Lidar2_z) * cos(RobotLidar_z)) + (0.11 * sin(Lidar1Lidar2_z) * sin(RobotLidar_z))) -
    //        Robot1Robot2_y * ((0.11 * cos(Lidar1Lidar2_z) * sin(RobotLidar_z)) - (0.11 * cos(RobotLidar_z) * sin(Lidar1Lidar2_z)));

    double t2 = (Lidar1Lidar2_y + (11 * cos(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100 - (11 * cos(RobotLidar_z) * sin(Lidar1Lidar2_z)) / 100);
    double t1 = ((11 * cos(Lidar1Lidar2_z) * cos(RobotLidar_z)) / 100 - Lidar1Lidar2_x + (11 * sin(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100);
    t1        = t1 * t1;

    d(0) = 1 -
           (((11 * cos(Lidar1Lidar2_z) * cos(RobotLidar_z)) / 100 + (11 * sin(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100) /
                ((11 * cos(Lidar1Lidar2_z) * cos(RobotLidar_z)) / 100 - Lidar1Lidar2_x + (11 * sin(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100) +
            (((11 * cos(Lidar1Lidar2_z) * sin(RobotLidar_z)) / 100 - (11 * cos(RobotLidar_z) * sin(Lidar1Lidar2_z)) / 100) *
             (t2) / t1) /
                (t2 * t2 / t1 + 1));

    return d;
  };

  cppopt::Matrix x(1, 1);
  x(0) = dutorad(0);

  // Iterate while norm of the first order derivative is greater than some predefined threshold.
  cppopt::ResultInfo ri = cppopt::SUCCESS;
  int count             = 0;
  while (ri == cppopt::SUCCESS && df(x).norm() > 0.0005f && count < 100) {
    ri   = cppopt::newtonRaphson(f, df, x);
    x(0) = angleLimit(x(0));
    std::cout
        << count
        << std::fixed << std::setw(3)
        << " Parameters: du:" << radtodu(x.transpose())
        << " Error: " << f(x).norm()
        << std::endl;
    count++;
  }

  std::cout << "Found a " << (f(x)(0) < 0.f ? "Maximum" : "Minimum") << std::endl;

  assert(fabs(x(0) - cppopt::Scalar(3.841)) < cppopt::Scalar(0.001));

  return 0;
}