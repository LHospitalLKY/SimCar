//
// Created by LHospital
//

#include "../include/ResidualFunction.h"

#include <iostream>

ResidualFunction::ResidualFunction(CostFunction *cost_fun, int METHOD) {
    cost_function_ = cost_fun;
    METHOD_ = METHOD;
}

Eigen::MatrixXd ResidualFunction::computeHessian() {
    // 求cost function的jacobian与cost
    cost_function_ -> computeCost();
    cost_function_ -> computeJacobian();
    
    cost_ = cost_function_ -> returnCost();
    Jacobian_ = cost_function_ -> returnJacobian();

    // 求矩阵W
    computeKernelDerivative(cost_);
    computeW(cost_);

    H_ = Jacobian_.transpose()*W_*Jacobian_;

    // LOG(INFO) << "size H: " << H_.rows() << " " << H_.cols();
    // LOG(INFO) << "H: \n" << H_;

    return H_;
}

Eigen::VectorXd ResidualFunction::computeBias() {
    B_ = derivative_*Jacobian_.transpose()*cost_;

    // LOG(INFO) << "size of B: " << B_.rows();
    // LOG(INFO) << "B: " << B_; 

    return B_;
}

Eigen::VectorXd ResidualFunction::getCost() {
    return cost_;
}

void ResidualFunction::changeParameters(const Eigen::VectorXd &params) {
    // 判断长度是否相同
    if(cost_function_ -> num_of_parameters_ == params.rows()) {
        cost_function_ -> parameters_ = params;
    }
}

void ResidualFunction::computeKernelDerivative(const Eigen::VectorXd &cost) {
    double s = cost.dot(cost);
    switch(METHOD_) {
        case 0:
            derivative_ = 1;
            dderivative_ = 0;
            break;
        case 1:
            derivative_ = 1/(1 + s/std::pow(CONTROL_PARA, 2));
            dderivative_ = -(1/std::pow(CONTROL_PARA, 2)) * std::pow(derivative_, 2);
    }
}

void ResidualFunction::computeW(const Eigen::VectorXd &cost) {
    Eigen::MatrixXd D, DD;
    DD = 2*dderivative_*cost*cost.transpose();
    int rows = cost.rows();
    D = derivative_*Eigen::MatrixXd::Identity(rows, rows);
    // LOG(INFO) << "DD: \n" << DD;

    W_ = DD + D;
}