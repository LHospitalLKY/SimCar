//
// Created by LHospital
//
#ifndef _RESIDUALFUNCTION_H
#define _RESIDUALFUNCTION_H 

#include "CostFunction.h"
#include <eigen3/Eigen/Eigen>
// #include <glog/logging.h>

#define RE_LINEAR 0
#define CAUCHY 1
#define CONTROL_PARA 1  // ATTENTION: 柯西核函数控制参数暂定为1

class ResidualFunction {
public:
    ResidualFunction(CostFunction *cost_fun, int METHOD);
    // 计算Hessian矩阵
    Eigen::MatrixXd computeHessian();
    Eigen::VectorXd computeBias();
    Eigen::VectorXd getCost();
    void changeParameters(const Eigen::VectorXd &params);             // 更新paramters

private:
    // 求解鲁棒核函数的导数与二阶导
    void computeKernelDerivative(const Eigen::VectorXd &cost);
    void computeW(const Eigen::VectorXd &cost);

private:
    CostFunction* cost_function_;
    double derivative_;                   // 鲁棒核函数的一阶导
    double dderivative_;                  // 鲁棒核函数的二阶导
    Eigen::VectorXd cost_;
    Eigen::MatrixXd Jacobian_;
    Eigen::MatrixXd W_;
    Eigen::MatrixXd H_;
    Eigen::VectorXd B_;
    int METHOD_;
};

#endif	// _RESIDUALFUNCTION_H