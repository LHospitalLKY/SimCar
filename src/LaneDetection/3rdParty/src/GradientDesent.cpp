//
// Created by LHospital
//

#include "../include/GradientDesent.h"

#include <iostream>

GradientDesent::GradientDesent(Eigen::VectorXd &param_init, int num_of_params, int dimension_of_cost, int method) {
    method_ = method;
    params_ = param_init;
    Hessian_ = Eigen::MatrixXd::Zero(num_of_params, num_of_params);
    Bias_ = Eigen::VectorXd::Zero(num_of_params);
    Cost_ = 0;

    iteration_ = 100;
}

void GradientDesent::addResidualBlock(ResidualFunction* res_block) {
    res_blocks_.push_back(res_block);
}

void GradientDesent::solve() {
    if(res_blocks_.size() == 0) {
        std::cerr << "No Residual block in this optimizer!";
        return;
    }
    if(method_ == GN) {
        // Gauss-Newton法
        // std::cout << "Begin to optimize~!";
        double lastCost = Cost_;
        for(int iter = 0; iter < iteration_; iter++) {
            // Hessian矩阵和Bias清零
            Hessian_ = Eigen::MatrixXd::Zero(Hessian_.rows(), Hessian_.rows());
            Bias_ = Eigen::VectorXd::Zero(Bias_.rows());
            // Cost_清零
            Cost_ = 0;
            ResidualFunction *res_block_tmp;
            for(int i = 0; i < res_blocks_.size(); i++) {
                // TODO: 调试结束之后化简代码
                res_block_tmp = res_blocks_[i];
                Hessian_ = Hessian_ + res_block_tmp -> computeHessian();
                Bias_ = Bias_ + res_block_tmp -> computeBias();
                Cost_ = Cost_ + res_block_tmp -> getCost().dot(res_block_tmp -> getCost());
            }

            // std::cout << "Hessian: \n" << Hessian_;
            // std::cout << "Bias: \n" << Bias_;

            Eigen::VectorXd dx = Hessian_.ldlt().solve(-Bias_);
            if(std::isnan(dx[0])) {
                std::cerr << "result is nan!";
                break;
            }
            if(iter > 0 && Cost_ >= lastCost) {
                // std::cout << "cost: " << Cost_;
                // std::cout << "last cost: " << lastCost;
                break;
            }

            params_ += dx;
            // 更新params
            for(int i = 0; i < res_blocks_.size(); i++) {
                // TODO: 调试结束之后化简代码
                res_block_tmp = res_blocks_[i];
                res_block_tmp -> changeParameters(params_);
            }

            lastCost = Cost_;
            // std::cout << "params in " << iter << "iterate: \n" << params_ << std::endl;
        }
        
        // std::cout << "Final result: \n" << params_ << std::endl;

        return;
    }

    if(method_ == LM) {
        // Levenberg-Marquardt法
        return;
    }
}

Eigen::VectorXd GradientDesent::getParams() {
    return params_;
}