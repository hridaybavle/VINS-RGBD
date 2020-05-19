#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>

class AutoDiffVelCostFunctor {

public:
  AutoDiffVelCostFunctor()
  {

  }

  template <typename T>
  bool operator()(const T* const x,
                  const T* const y,
                  T* residuals) const {

    double var = 0.09;
    double var_z = 0.09;
    residuals[0] = (x[0] -  y[0])/ var;
    residuals[1] = (x[1] -  y[1])/ var;
    residuals[2] = (x[2] -  y[2])/ var_z;

    //std::cout << "estimated v: " << x[0] << std::endl << x[1] << std::endl << x[2] << std::endl;
    //std::cout << "meas v: " << y[0] << std::endl << y[1] << std::endl << y[2] << std::endl;
    //std::cout << "vel residuals: " << residuals[0] << std::endl << residuals[1] << std::endl << residuals[2] << std::endl;

    return true;
  }

};
