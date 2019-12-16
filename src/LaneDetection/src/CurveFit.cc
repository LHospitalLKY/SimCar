
#include "../include/CurveFit.h"

bool QuadFitting::solve(std::vector<double> &abc) {
    // 设置初值
    Eigen::VectorXd parameters(3);
    Eigen::VectorXd residuals(1);
    Eigen::VectorXd constants(2);
    // parameters的初始值
    parameters << 0, 0, 0;
    
    Quad_CostFunction *cost_function = nullptr;
    ResidualFunction *residual_block = nullptr;
    GradientDesent gd(parameters, 3, 1, GN);

    for(int i = 0; i < x_ -> size(); i++) {
        // std::cout << "x, y: " << (*x_)[i] << " " << (*y_)[i];
        constants << (*y_)[i], (*x_)[i];
        // std::cout << "Constant: " << constants;
        cost_function = new Quad_CostFunction(&parameters, &residuals, 3, 1, &constants, 2);

        // std::cout << "constant in cost_function: " << cost_function -> constants_;
        // cost_vector.push_back(cost_function);
        residual_block = new ResidualFunction(cost_function, 0);
        
        gd.addResidualBlock(residual_block);
    }

    gd.solve();

    Eigen::VectorXd params = gd.getParams();

    abc.push_back(params(0));
    abc.push_back(params(1));
    abc.push_back(params(2));

    return true;
}