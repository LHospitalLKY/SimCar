//
// Created by LHospital
//
#ifndef _GRADIENDESENT_H
#define _GRADIENDESENT_H 

#include "ResidualFunction.h"

#include <vector>
// #include <glog/logging.h>
#include <eigen3/Eigen/Eigen>

/**
 * @brief: 关键名称(参考ceres名词解释)
 * 1. 最小二乘问题构建为 min_{x} 1/2 * \sum_{i} \rho_i(||f_{i}(x_i, ... , x_ik)||^2)
 * 2. \rho_i(||f_{i}(x_1, ... , x_ik)||^2称为ResidualBlock
 * 3. f_{i}(x_1, ... , x_ik)称为CostFunction
 * 4. [x_1, ... , x_ik]称为parameter block
 * 5. \rho(*)称为KernelFunction(Ceres中称为LossFunction)
*/

// TODO: 优化代码，搞定初值的赋值问题，减少重复赋值
// TODO: 更改变量名，分清f(x)的函数值与||f(x)||^2的名称
// TODO: 复杂代码优化，相比结构化程序，

#define GN 0
#define LM 1

class GradientDesent {
public:
    GradientDesent(Eigen::VectorXd &param_init, int num_of_params, int dimension_of_cost, int method_ = GN);
    // GradientDesent* createLMOptimizer();
    void addResidualBlock(ResidualFunction* res_block);
    void solve();
    Eigen::VectorXd getParams();
private:
    int method_;
    int iteration_;                       // 迭代次数
    std::vector<ResidualFunction*> res_blocks_;
    Eigen::VectorXd params_;
    Eigen::MatrixXd Hessian_;
    Eigen::VectorXd Bias_;
    double Cost_;
};


#endif	// _GRADIENDESENT_H