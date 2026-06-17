/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <vins/utility/utility.h>

#include <Eigen/Dense>

class PoseLocalParameterization : public ceres::Manifold
{
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const;
  virtual bool PlusJacobian(const double *x, double *jacobian) const;
  // Minus / MinusJacobian are part of the Manifold interface but are not used
  // by the VINS solve (only Plus / PlusJacobian are). They are provided so the
  // class is concrete; they intentionally report failure if ever invoked.
  virtual bool Minus(const double *y, const double *x,
                     double *y_minus_x) const;
  virtual bool MinusJacobian(const double *x, double *jacobian) const;
  virtual int AmbientSize() const { return 7; };
  virtual int TangentSize() const { return 6; };
};
