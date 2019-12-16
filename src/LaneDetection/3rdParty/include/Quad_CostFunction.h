//
// Created by LHospital
//
#ifndef _QUAD_COSTFUNCTION_H
#define _QUAD_COSTFUNCTION_H 

// #include "Jacobian.h"
#include "../include/CostFunction.h"
#include "../include/ResidualFunction.h"
#include "../include/GradientDesent.h"

#include <iostream>
#include <ctime>
#include <chrono>
#include <vector>
// #include <glog/logging.h>
// #include <gtest/gtest.h>

class Quad_CostFunction : public CostFunction {
public:
    Quad_CostFunction(Eigen::VectorXd* parameters, Eigen::VectorXd* residuals, int num_of_parameters, int num_of_residuals, Eigen::VectorXd* constants, int num_of_constants) : CostFunction(parameters, residuals, num_of_parameters, num_of_residuals, constants, num_of_constants) {}

    void computeCost();
    void computeJacobian();
};

#endif	// _QUAD_COSTFUNCTION_H