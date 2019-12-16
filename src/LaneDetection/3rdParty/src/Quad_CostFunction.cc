//
// Created by LHospital
//
#include "../include/Quad_CostFunction.h"

// 本问题中求和号中的cost function不同
void Quad_CostFunction::computeCost() {
    // constants的顺序为y, x
    (residuals_)(0) =  (constants_)(0) - ((parameters_)(0)*pow((constants_)(1), 2) + (parameters_)(1)*(constants_)(1) + (parameters_)(2));

    // LOG(INFO) << "residual: " << (residuals_).transpose();
}

void Quad_CostFunction::computeJacobian() {
    // 计算Jacobian矩阵
    Jacobian_(0) = -pow((constants_)(1), 2);
    Jacobian_(1) = -(constants_)(1);
    Jacobian_(2) = -1;

    // LOG(INFO) << "Jacobian: " << Jacobian_;
}

