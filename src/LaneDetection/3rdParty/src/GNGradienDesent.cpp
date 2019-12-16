//
// Created by LHospital
//

#include "../include/GNGradientDesent.h"

#include <iostream>

GNOptimizer::GNOptimizer(Jacobian *jacobian, const Eigen::VectorXd &x_init) : GDOptimizer() {
    this -> jacobian_ = jacobian;
    this -> x_ = x_init;
}

void GNOptimizer::computeJacobian() {
    // jacobian_ -> 
}
